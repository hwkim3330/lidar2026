/**
 * lidar-proxy.js — UDP → WebSocket bridge for Ouster LiDAR
 *
 * Supports multiple LiDAR instances, each on a different UDP port
 * and WebSocket path. Backward-compatible: /ws/lidar still works.
 *
 * Wire format to browser (per frame, Float32Array):
 *   [x0, y0, z0, intensity0, x1, y1, z1, intensity1, ...]
 */

import dgram from 'dgram';

// Ouster OS-1-16 Gen1 beam intrinsics (from get_beam_intrinsics)
const BEAM_ALTITUDE_DEG = [14.18, 12.06, 9.95, 8.70, 5.80, 3.73, 1.67, -0.40,
                           -2.47, -4.54, -6.61, -8.69, -10.78, -12.89, -15.02, -17.21];
const BEAM_AZIMUTH_DEG  = [-3.30, -3.26, -3.21, -3.16, -3.15, -3.13, -3.11, -3.09,
                           -3.08, -3.07, -3.07, -3.08, -3.07, -3.09, -3.11, -3.15];
const ORIGIN_MM = 12.163;
const PIXELS_PER_COL = 16;
const ENCODER_TICKS = 90112;

const beamAlt = BEAM_ALTITUDE_DEG.map(d => d * Math.PI / 180);
const beamAz  = BEAM_AZIMUTH_DEG.map(d => d * Math.PI / 180);
const cosAlt  = beamAlt.map(Math.cos);
const sinAlt  = beamAlt.map(Math.sin);

/**
 * Parse LEGACY lidar packet (3392 bytes = 16 columns × 212 bytes)
 */
function parsePacket(buf) {
  const COL_SIZE = 212;
  const cols = [];
  for (let c = 0; c < 16; c++) {
    const off = c * COL_SIZE;
    const measId = buf.readUInt16LE(off + 8);
    const frameId = buf.readUInt16LE(off + 10);
    const encoder = buf.readUInt32LE(off + 12);
    const azRad = (encoder / ENCODER_TICKS) * 2 * Math.PI;

    const pixels = [];
    for (let p = 0; p < PIXELS_PER_COL; p++) {
      const poff = off + 16 + p * 12;
      const range = buf.readUInt32LE(poff);
      const refl  = buf.readUInt16LE(poff + 4);
      const signal = buf.readUInt16LE(poff + 6);

      if (range === 0) { pixels.push(null); continue; }

      const r = range / 1000;
      const totalAz = azRad + beamAz[p];
      const x = r * cosAlt[p] * Math.cos(totalAz) + (ORIGIN_MM / 1000) * Math.cos(totalAz);
      const y = r * cosAlt[p] * Math.sin(totalAz) + (ORIGIN_MM / 1000) * Math.sin(totalAz);
      const z = r * sinAlt[p];

      pixels.push({ x, y, z, intensity: refl || signal, range, row: p });
    }
    cols.push({ measId, frameId, encoder, pixels });
  }
  return cols;
}

/**
 * Create a single LiDAR UDP→WS instance
 * @param {object} wss - WebSocketServer
 * @param {string} id - LiDAR identifier
 * @param {number} udpPort - UDP port to listen on
 * @param {string[]} wsPaths - WebSocket paths to serve (e.g. ['/ws/lidar-a', '/ws/lidar'])
 */
function createLidarInstance(wss, id, udpPort, wsPaths) {
  const udp = dgram.createSocket({ type: 'udp4', reuseAddr: true });

  let currentFrameId = -1;
  let framePoints = [];
  let clients = new Set();
  let stats = { fps: 0, points: 0, lastFrame: 0, frames: 0 };
  let lastStatTime = Date.now();

  let recorder = null; // { write(payload, {dstPort}), close() }
  let profileDecorator = null; // (profile) => profile  — optional mutator run before WS broadcast / API

  // Track WS clients for this instance
  let timingClients = new Set();

  // ── Tracking (background subtraction) ──
  // Range image: 16 beams × 512 cols (LEGACY 512x10). One Uint16Array[8192].
  const N_COLS = 512;
  const GRID = PIXELS_PER_COL * N_COLS;
  const rangeCurrent = new Uint16Array(GRID);  // this-frame range in cm (range_mm >> 4 clipped to 65535)
  const bgMax = new Uint16Array(GRID);         // running max-range = background estimate
  let bgLearnedFrames = 0;
  let trackingClients = new Set();
  const TRACK_THRESH_MM = 400;  // foreground if current < bg - 400mm
  const TRACK_MIN_BG_MM = 500;  // ignore cells with no background yet
  let trackingEnabled = false;

  function resetBackground() {
    bgMax.fill(0);
    bgLearnedFrames = 0;
  }

  wss.on('connection', (ws, req) => {
    if (wsPaths.includes(req.url)) {
      clients.add(ws);
      ws.send(JSON.stringify({
        type: 'metadata',
        id,
        beams: PIXELS_PER_COL,
        beamAltitude: BEAM_ALTITUDE_DEG,
        beamAzimuth: BEAM_AZIMUTH_DEG,
        originMm: ORIGIN_MM
      }));
      ws.on('close', () => clients.delete(ws));
    }
    // Timing WS: streams packet arrival timestamps
    const timingPath = wsPaths[0].replace('/ws/lidar', '/ws/lidar-timing');
    if (req.url === timingPath) {
      timingClients.add(ws);
      ws.on('close', () => timingClients.delete(ws));
    }
    // Tracking WS: foreground-only cloud (after background subtraction)
    const trackPath = wsPaths[0].replace('/ws/lidar', '/ws/lidar-track');
    if (req.url === trackPath) {
      trackingClients.add(ws);
      trackingEnabled = true;
      ws.send(JSON.stringify({ type: 'track-config', thresholdMm: TRACK_THRESH_MM, bgFrames: bgLearnedFrames }));
      ws.on('message', (raw) => {
        try { const m = JSON.parse(raw); if (m.cmd === 'reset-bg') resetBackground(); } catch {}
      });
      ws.on('close', () => {
        trackingClients.delete(ws);
        if (trackingClients.size === 0) trackingEnabled = false;
      });
    }
  });

  function broadcastFrame(points) {
    stats.frames++;
    stats.points = points.length;
    const now = Date.now();
    if (now - lastStatTime >= 1000) {
      stats.fps = stats.frames;
      stats.frames = 0;
      lastStatTime = now;
    }
    if (clients.size === 0) return;
    const buf = new Float32Array(points.length * 4);
    for (let i = 0; i < points.length; i++) {
      const p = points[i];
      buf[i * 4]     = p.x;
      buf[i * 4 + 1] = p.y;
      buf[i * 4 + 2] = p.z;
      buf[i * 4 + 3] = p.intensity;
    }
    const binary = Buffer.from(buf.buffer);
    for (const ws of clients) {
      if (ws.readyState === 1) ws.send(binary);
    }
  }

  function broadcastForeground(points) {
    // Update background (running max over learned frames); emit foreground cloud
    const fg = [];
    const needLearn = bgLearnedFrames < 30;
    for (let i = 0; i < points.length; i++) {
      const p = points[i];
      const idx = p.gridIdx;
      const r = p.range; // mm
      if (needLearn) {
        if (r > bgMax[idx]) bgMax[idx] = Math.min(r, 65535);
        continue;
      }
      const bg = bgMax[idx];
      if (bg < TRACK_MIN_BG_MM) {
        if (r > bg) bgMax[idx] = Math.min(r, 65535);
        continue;
      }
      // Foreground condition
      if (r < bg - TRACK_THRESH_MM) {
        fg.push(p);
      } else if (r > bg) {
        // Farther than previous max → grow bg slightly (scene expanded)
        bgMax[idx] = Math.min(r, 65535);
      }
    }
    bgLearnedFrames++;

    const header = JSON.stringify({ type: 'fg-meta', fgCount: fg.length, bgFrames: bgLearnedFrames });
    const buf = new Float32Array(fg.length * 4);
    for (let i = 0; i < fg.length; i++) {
      const p = fg[i];
      buf[i * 4] = p.x; buf[i * 4 + 1] = p.y; buf[i * 4 + 2] = p.z; buf[i * 4 + 3] = p.intensity;
    }
    const binary = Buffer.from(buf.buffer);
    for (const ws of trackingClients) {
      if (ws.readyState === 1) { ws.send(header); ws.send(binary); }
    }
  }

  // ── Traffic profiling ──
  let lastLockedCycle = 0; // locked cycle to prevent jitter flapping
  const PROFILE_WINDOW = 100; // track last N frames
  let pktCount = 0;
  let pktTimestamps = [];      // per-packet arrival times (µs precision)
  let frameTimestamps = [];    // per-frame completion times
  let framePktCounts = [];     // packets per frame
  let frameByteCounts = [];    // bytes per frame
  let currentFramePkts = 0;
  let currentFrameBytes = 0;

  function recordFrameProfile() {
    const now = process.hrtime.bigint(); // nanoseconds
    frameTimestamps.push(Number(now) / 1000); // µs
    framePktCounts.push(currentFramePkts);
    frameByteCounts.push(currentFrameBytes);
    if (frameTimestamps.length > PROFILE_WINDOW) {
      frameTimestamps.shift();
      framePktCounts.shift();
      frameByteCounts.shift();
    }
    currentFramePkts = 0;
    currentFrameBytes = 0;
  }

  function getTrafficProfile() {
    if (frameTimestamps.length < 3) return null;

    // Frame intervals
    const intervals = [];
    for (let i = 1; i < frameTimestamps.length; i++) {
      intervals.push(frameTimestamps[i] - frameTimestamps[i - 1]);
    }
    const avgInterval = intervals.reduce((a, b) => a + b, 0) / intervals.length;
    const minInterval = Math.min(...intervals);
    const maxInterval = Math.max(...intervals);
    const jitter = Math.sqrt(intervals.reduce((s, v) => s + (v - avgInterval) ** 2, 0) / intervals.length);

    const avgPktsPerFrame = framePktCounts.reduce((a, b) => a + b, 0) / framePktCounts.length;
    const avgBytesPerFrame = frameByteCounts.reduce((a, b) => a + b, 0) / frameByteCounts.length;
    const fps = avgInterval > 0 ? 1e6 / avgInterval : 0;
    const bandwidthMbps = avgBytesPerFrame * fps * 8 / 1e6;

    // Per-packet intervals (within recent packets)
    const pktIntervals = [];
    for (let i = 1; i < pktTimestamps.length; i++) {
      pktIntervals.push(pktTimestamps[i] - pktTimestamps[i - 1]);
    }
    const avgPktInterval = pktIntervals.length > 0 ? pktIntervals.reduce((a, b) => a + b, 0) / pktIntervals.length : 0;

    // Burst analysis: how long does one frame's packets take to arrive
    // Approximate: avgPktsPerFrame * avgPktInterval
    const burstDurationUs = avgPktsPerFrame * avgPktInterval;

    // Per-packet jitter (σ of rx inter-arrival)
    const pktJitter = pktIntervals.length > 1
      ? Math.sqrt(pktIntervals.reduce((s, v) => s + (v - avgPktInterval) ** 2, 0) / pktIntervals.length)
      : 0;

    // NOTE: All *Us below are captured at Node UDP recv time via
    // process.hrtime.bigint() inside udp.on('message') — kernel→Node boundary.
    // They do NOT include WebSocket serialization, browser event-loop, or
    // render latency. Client must compute its own render jitter separately.
    const stamp = 'rx';
    return {
      measuredAt: stamp,
      fps: Math.round(fps * 10) / 10,
      // rx-stamped frame timing
      rxFrameIntervalUs: Math.round(avgInterval),
      rxFrameIntervalMinUs: Math.round(minInterval),
      rxFrameIntervalMaxUs: Math.round(maxInterval),
      rxFrameJitterUs: Math.round(jitter),
      // rx-stamped packet timing
      rxPktIntervalUs: Math.round(avgPktInterval),
      rxPktJitterUs: Math.round(pktJitter),
      // volume
      pktsPerFrame: Math.round(avgPktsPerFrame),
      bytesPerFrame: Math.round(avgBytesPerFrame),
      pktSize: 3392,
      burstDurationUs: Math.round(burstDurationUs),
      bandwidthMbps: Math.round(bandwidthMbps * 100) / 100,
      samples: frameTimestamps.length,
      // ── Back-compat aliases (deprecated; prefer rx* names) ──
      frameIntervalUs: Math.round(avgInterval),
      frameIntervalMinUs: Math.round(minInterval),
      frameIntervalMaxUs: Math.round(maxInterval),
      jitterUs: Math.round(jitter),
      pktIntervalUs: Math.round(avgPktInterval),
    };
  }

  /**
   * Generate optimal TAS config derived from observed LiDAR packet pattern.
   *
   * The cycle time is NOT arbitrary — it comes from the measured inter-packet
   * interval. Each cycle handles exactly 1 LiDAR packet.
   *
   * @param {object} opts
   * @param {number} opts.cycleUs - Override cycle (default: auto from packet interval)
   * @param {number} opts.marginFactor - Jitter margin multiplier (default: 2.0)
   * @param {number} opts.linkMbps - Link rate (default: 1000)
   */
  function generateTasConfig(opts = {}) {
    const profile = getTrafficProfile();
    if (!profile) return null;

    const linkMbps = opts.linkMbps || 1000;
    const marginFactor = opts.marginFactor || 2.0;

    // ── Derive cycle from packet pattern ──
    // Cycle = measured packet interval, only lock after 50+ frames for stability
    const naturalCycleUs = profile.pktIntervalUs;
    if (profile.samples >= 50) {
      if (lastLockedCycle === 0 || Math.abs(naturalCycleUs - lastLockedCycle) > 10) {
        lastLockedCycle = Math.round(naturalCycleUs);
      }
    }
    const cycleUs = opts.cycleUs || (lastLockedCycle > 0 ? lastLockedCycle : Math.round(naturalCycleUs));

    // ── Per-packet wire time ──
    const pktBytes = profile.pktSize + 20 + 14 + 4; // IP + Ethernet + FCS
    const pktTxUs = pktBytes * 8 / linkMbps;         // transmission time in µs

    // ── LiDAR slot: tx time + jitter margin ──
    const lidarSlotUs = Math.round(pktTxUs * marginFactor * 10) / 10;
    const guardUs = 1;
    const beSlotUs = Math.round((cycleUs - lidarSlotUs - guardUs * 2) * 10) / 10;

    if (beSlotUs < 1) {
      return { error: 'Cycle too short for packet', profile, cycleUs, pktTxUs, lidarSlotUs };
    }

    // ── Packet timing analysis ──
    // Compute inter-packet jitter from raw timestamps
    const pktJitterUs = pktTimestamps.length > 2 ? (() => {
      const ints = [];
      for (let i = 1; i < pktTimestamps.length; i++) ints.push(pktTimestamps[i] - pktTimestamps[i-1]);
      const mean = ints.reduce((a,b) => a+b,0) / ints.length;
      return Math.round(Math.sqrt(ints.reduce((s,v) => s + (v-mean)**2, 0) / ints.length));
    })() : 0;

    return {
      derived: true,
      cycleUs,
      naturalCycleUs: Math.round(naturalCycleUs),
      pktTxUs: Math.round(pktTxUs * 100) / 100,
      entries: [
        { gateStates: 128, durationUs: lidarSlotUs, note: `TC7 LiDAR (1 pkt × ${marginFactor}x margin)` },
        { gateStates: 0, durationUs: guardUs, note: 'guard band' },
        { gateStates: 127, durationUs: beSlotUs, note: 'TC0-6 Best Effort' },
        { gateStates: 0, durationUs: guardUs, note: 'guard band' },
      ],
      utilization: Math.round(lidarSlotUs / cycleUs * 10000) / 100,
      packetAnalysis: {
        pktSize: profile.pktSize,
        pktTxUs: Math.round(pktTxUs * 100) / 100,
        pktIntervalUs: profile.pktIntervalUs,
        pktJitterUs,
        pktsPerFrame: profile.pktsPerFrame,
        frameIntervalUs: profile.frameIntervalUs,
        fps: profile.fps,
        bandwidthMbps: profile.bandwidthMbps,
      },
      profile,
    };
  }

  udp.on('message', (msg) => {
    const nowUs = Number(process.hrtime.bigint()) / 1000;
    pktCount++;
    if (pktCount <= 3) console.log(`  LiDAR [${id}] pkt #${pktCount}: ${msg.length} bytes`);
    if (msg.length !== 3392) return;

    // Track packet timestamps (keep last 500)
    pktTimestamps.push(nowUs);
    if (pktTimestamps.length > 500) pktTimestamps.shift();

    // Record raw packet to pcap if active
    if (recorder) {
      try { recorder.write(msg, { dstPort: udpPort }); } catch (e) { console.error(`[rec] ${e.message}`); }
    }

    currentFramePkts++;
    currentFrameBytes += msg.length;

    const cols = parsePacket(msg);
    if (cols.length === 0) return;

    const fid = cols[0].frameId;
    const isNewFrame = fid !== currentFrameId && currentFrameId !== -1;

    if (isNewFrame) {
      recordFrameProfile();
      broadcastFrame(framePoints);
      if (trackingEnabled) broadcastForeground(framePoints);
      framePoints = [];
    }
    currentFrameId = fid;

    // Stream timing data to timing clients
    if (timingClients.size > 0) {
      const pts = cols.reduce((s, c) => s + c.pixels.filter(p => p).length, 0);
      // Per-packet event
      const pktMsg = JSON.stringify({
        type: 'pkt',
        t: Math.round(nowUs),
        fid, sz: msg.length, pts,
        newFrame: isNewFrame,
      });
      for (const ws of timingClients) {
        if (ws.readyState === 1) ws.send(pktMsg);
      }

      // On new frame: send real-time profile + auto TAS
      if (isNewFrame && frameTimestamps.length >= 5) {
        // Throttle: only every 10 frames (~1/sec at 10fps)
        if (stats.frames % 10 === 0) {
          const prof = getTrafficProfile();
          const tas = generateTasConfig();
          if (prof) {
            if (profileDecorator) try { profileDecorator(prof); } catch (e) { /* ignore */ }
            const summaryMsg = JSON.stringify({
              type: 'profile',
              profile: prof,
              autoTas: tas,
            });
            for (const ws of timingClients) {
              if (ws.readyState === 1) ws.send(summaryMsg);
            }
          }
        }
      }
    }

    for (const col of cols) {
      for (const px of col.pixels) {
        if (px) {
          px.gridIdx = px.row * N_COLS + (col.measId % N_COLS);
          framePoints.push(px);
        }
      }
    }
  });

  udp.on('error', (err) => {
    console.error(`  LiDAR [${id}] UDP error:`, err.message);
  });

  udp.bind(udpPort, '0.0.0.0', () => {
    console.log(`  LiDAR [${id}]: UDP :${udpPort} → WebSocket ${wsPaths.join(', ')}`);
  });

  /**
   * Snapshot current packet timing data for capture/export
   */
  function getTimingSnapshot() {
    if (pktTimestamps.length < 10) return null;
    // Compute relative timestamps (offset from first)
    const t0 = pktTimestamps[0];
    const relTimestamps = pktTimestamps.map(t => Math.round((t - t0) * 100) / 100);

    // Per-packet intervals
    const intervals = [];
    for (let i = 1; i < pktTimestamps.length; i++) {
      intervals.push(Math.round((pktTimestamps[i] - pktTimestamps[i - 1]) * 100) / 100);
    }

    return {
      count: pktTimestamps.length,
      timestamps_us: relTimestamps,
      intervals_us: intervals,
      frameIntervals_us: frameTimestamps.length > 1
        ? frameTimestamps.slice(1).map((t, i) => Math.round((t - frameTimestamps[i]) * 100) / 100)
        : [],
    };
  }

  return {
    id,
    getStats: () => ({ ...stats, id, clients: clients.size, recording: !!recorder }),
    getTrafficProfile,
    generateTasConfig,
    getTimingSnapshot,
    setRecorder: (r) => { recorder = r; },
    getRecorder: () => recorder,
    setProfileDecorator: (fn) => { profileDecorator = fn; },
  };
}

/**
 * Setup all LiDAR proxies from config
 * @param {object} server - HTTP server
 * @param {object} wss - WebSocketServer
 * @param {object[]} lidarConfigs - Array of { id, udpPort, wsPath, label }
 * @param {string} [defaultWsPath] - Backward-compat path (e.g. '/ws/lidar')
 */
export function setupLidarProxy(server, wss, lidarConfigs, defaultWsPath) {
  // Single-LiDAR backward compat: if no config array, use legacy defaults
  if (!lidarConfigs) {
    lidarConfigs = [{ id: 'lidar', udpPort: 7502, wsPath: '/ws/lidar' }];
  }

  const instances = lidarConfigs.map((cfg, i) => {
    const wsPaths = [cfg.wsPath];
    // First LiDAR also serves the default path for backward compat
    if (i === 0 && defaultWsPath && defaultWsPath !== cfg.wsPath) {
      wsPaths.push(defaultWsPath);
    }
    return createLidarInstance(wss, cfg.id, cfg.udpPort, wsPaths);
  });

  return {
    instances,
    getStats: () => instances.map(inst => inst.getStats()),
    // Backward compat: single stats for first instance
    getFirstStats: () => instances[0]?.getStats() || { fps: 0, points: 0, frames: 0, clients: 0 }
  };
}
