/**
 * kernel-rx.js — True kernel receive-timestamp jitter via tcpdump (libpcap SO_TIMESTAMP).
 *
 * Node's `udp.on('message')` timestamp (process.hrtime.bigint()) fires only when
 * the event loop dequeues the datagram — under CPU load the event loop stalls and
 * the measurement inherits that stall. Kernel-level pcap timestamps are set in the
 * network stack (skbuff->tstamp) at packet arrival and are NOT affected by user-
 * space scheduling.
 *
 * Requires tcpdump with CAP_NET_RAW + CAP_NET_ADMIN:
 *   sudo setcap cap_net_raw,cap_net_admin=eip /usr/bin/tcpdump
 */

import { spawn } from 'child_process';

export function startKernelRx({ iface, port, pktsPerFrame = 32 } = {}) {
  // -l : line-buffered, -n : no name resolution, --micro : µs resolution, -tt : abs time (seconds.micros),
  // -q : brief output (1 line per packet)
  const filter = `udp port ${port}`;
  const args = ['-l', '-n', '-q', '--micro', '-tt', '-i', iface, filter];
  const proc = spawn('tcpdump', args, { stdio: ['ignore', 'pipe', 'pipe'] });

  const pktTs = [];          // kernel timestamps in µs (last 500)
  const frameTs = [];        // every pktsPerFrame-th packet = proxy for frame start (last 100)
  let pktCounter = 0;
  let lastPktUs = 0;
  let stderrBuf = '';

  proc.stderr.on('data', (d) => { stderrBuf += d.toString(); });
  proc.on('error', (e) => console.error('[krx] tcpdump error:', e.message));
  proc.on('exit', (code) => console.log(`[krx] tcpdump exited ${code} — ${stderrBuf.slice(-300)}`));

  let lineBuf = '';
  proc.stdout.on('data', (chunk) => {
    lineBuf += chunk.toString();
    let nl;
    while ((nl = lineBuf.indexOf('\n')) >= 0) {
      const line = lineBuf.slice(0, nl); lineBuf = lineBuf.slice(nl + 1);
      // Match leading float timestamp: "1734547200.123456 IP ..."
      const m = line.match(/^(\d+)\.(\d+)\s+IP/);
      if (!m) continue;
      const secs = parseInt(m[1], 10);
      const frac = m[2].padEnd(6, '0').slice(0, 6); // normalize to µs
      const tUs = secs * 1e6 + parseInt(frac, 10);

      pktTs.push(tUs);
      if (pktTs.length > 500) pktTs.shift();

      // Ouster sends 32 packets per frame at constant spacing → use every Nth
      // packet as a frame-aligned reference point. After the first N packets
      // this converges to the true frame period with N-sample averaging.
      if (pktCounter % pktsPerFrame === 0) {
        frameTs.push(tUs);
        if (frameTs.length > 100) frameTs.shift();
      }
      pktCounter++;
      lastPktUs = tUs;
    }
  });

  function sigma(arr) {
    if (arr.length < 2) return 0;
    const d = [];
    for (let i = 1; i < arr.length; i++) d.push(arr[i] - arr[i - 1]);
    const mean = d.reduce((a, b) => a + b, 0) / d.length;
    return Math.sqrt(d.reduce((s, v) => s + (v - mean) ** 2, 0) / d.length);
  }

  function getStats() {
    const pktSigma = sigma(pktTs);
    const frameSigma = sigma(frameTs);
    return {
      krxPktJitterUs: Math.round(pktSigma),
      krxFrameJitterUs: Math.round(frameSigma),
      krxPktSamples: pktTs.length,
      krxFrameSamples: frameTs.length,
    };
  }

  function stop() { if (!proc.killed) proc.kill('SIGTERM'); }

  return { getStats, stop, proc };
}
