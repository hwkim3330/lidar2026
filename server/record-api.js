/**
 * record-api.js — PCAP record (native pcap writer) + replay via Ouster SDK.
 *
 * Record: the Node lidar-proxy streams every UDP payload into a PcapWriter,
 *         producing standard libpcap (DLT_EN10MB) files readable by Wireshark
 *         and ouster-sdk. A sidecar *.json sensor metadata is saved alongside.
 * Replay: spawn a Python script that re-emits pcap UDP to 127.0.0.1:<port>
 *         (the live lidar-proxy consumes it like real traffic).
 */

import { spawn } from 'child_process';
import path from 'path';
import fs from 'fs';
import { Router } from 'express';
import { lidars, paths } from './config.js';
import { PcapWriter } from './pcap-writer.js';

const ROOT = path.resolve(path.dirname(new URL(import.meta.url).pathname), '..');
const RECORD_DIR = path.join(ROOT, paths.recordingsDir);

const state = {
  record: null, // { writer, file, startedAt, lidarId, instance }
  replay: null, // { proc, file, startedAt }
};

function ensureDir() { fs.mkdirSync(RECORD_DIR, { recursive: true }); }
function stamp() { return new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19); }

async function fetchMetadata(host) {
  const urls = [
    `http://${host}/api/v1/sensor/metadata`,
    `http://${host}/api/v1/sensor/metadata/sensor_info`,
  ];
  try {
    const r = await fetch(urls[0], { signal: AbortSignal.timeout(5000) });
    if (r.ok) return await r.json();
  } catch (_) {}
  return null;
}

function listRecordings() {
  ensureDir();
  return fs.readdirSync(RECORD_DIR)
    .filter(f => f.endsWith('.pcap'))
    .map(f => {
      const full = path.join(RECORD_DIR, f);
      const st = fs.statSync(full);
      const json = f.replace(/\.pcap$/, '.json');
      return {
        name: f,
        sizeMB: Math.round(st.size / 1048576 * 100) / 100,
        mtime: st.mtime.toISOString(),
        hasMetadata: fs.existsSync(path.join(RECORD_DIR, json)),
      };
    })
    .sort((a, b) => b.mtime.localeCompare(a.mtime));
}

export default function recordApi(lidarRuntime) {
  const router = Router();

  router.get('/status', (req, res) => {
    const rec = state.record;
    res.json({
      record: rec && {
        file: path.basename(rec.file),
        startedAt: rec.startedAt,
        lidarId: rec.lidarId,
        packets: rec.writer.packetCount,
        sizeMB: Math.round(rec.writer.byteCount / 1048576 * 100) / 100,
      },
      replay: state.replay && {
        file: state.replay.file,
        startedAt: state.replay.startedAt,
        running: !state.replay.proc.killed,
      },
    });
  });

  router.get('/recordings', (req, res) => res.json(listRecordings()));

  router.delete('/recordings/:name', (req, res) => {
    const safe = req.params.name.replace(/[^\w.-]/g, '');
    if (!safe.endsWith('.pcap')) return res.status(400).json({ error: 'Invalid name' });
    const full = path.join(RECORD_DIR, safe);
    if (!fs.existsSync(full)) return res.status(404).json({ error: 'Not found' });
    fs.unlinkSync(full);
    const j = full.replace(/\.pcap$/, '.json');
    if (fs.existsSync(j)) fs.unlinkSync(j);
    res.json({ deleted: safe });
  });

  router.post('/record/start', async (req, res) => {
    if (state.record) return res.status(409).json({ error: 'Already recording', file: path.basename(state.record.file) });

    const lidarId = req.body?.lidarId || lidars[0].id;
    const lidar = lidars.find(l => l.id === lidarId);
    const inst = lidarRuntime.instances.find(i => i.id === lidarId);
    if (!lidar || !inst) return res.status(404).json({ error: `LiDAR not found: ${lidarId}` });

    ensureDir();
    const base = req.body?.name ? String(req.body.name).replace(/[^\w.-]/g, '') : `rec-${stamp()}`;
    const file = path.join(RECORD_DIR, `${base}.pcap`);

    // Fetch sensor metadata and save sidecar
    const meta = await fetchMetadata(lidar.host);
    if (meta) fs.writeFileSync(file.replace(/\.pcap$/, '.json'), JSON.stringify(meta, null, 2));

    const writer = new PcapWriter(file, { srcIp: lidar.host, dstIp: '192.168.1.1' });
    inst.setRecorder(writer);

    state.record = { writer, file, startedAt: new Date().toISOString(), lidarId, instance: inst };
    res.json({ started: true, file: path.basename(file), metadata: !!meta });
  });

  router.post('/record/stop', (req, res) => {
    if (!state.record) return res.status(409).json({ error: 'Not recording' });
    const { writer, file, instance } = state.record;
    instance.setRecorder(null);
    const { packets, bytes } = writer.close();
    state.record = null;
    res.json({
      stopped: true,
      file: path.basename(file),
      packets,
      sizeMB: Math.round(bytes / 1048576 * 100) / 100,
    });
  });

  router.post('/replay/start', (req, res) => {
    if (state.replay && !state.replay.proc.killed) {
      return res.status(409).json({ error: 'Already replaying', file: state.replay.file });
    }
    const name = String(req.body?.file || '').replace(/[^\w.-]/g, '');
    if (!name.endsWith('.pcap')) return res.status(400).json({ error: 'Invalid pcap name' });
    const file = path.join(RECORD_DIR, name);
    if (!fs.existsSync(file)) return res.status(404).json({ error: 'Not found' });

    const lidarId = req.body?.lidarId || lidars[0].id;
    const lidar = lidars.find(l => l.id === lidarId);
    if (!lidar) return res.status(404).json({ error: `LiDAR not found: ${lidarId}` });

    const rate = String(req.body?.rate || '1.0');
    const loopFlag = req.body?.loop ? '--loop' : '--no-loop';
    const py = path.resolve(ROOT, paths.python);
    const script = path.join(ROOT, 'scripts', 'replay.py');
    const args = [script, file, '--lidar-port', String(lidar.udpPort), '--imu-port', String(lidar.imuPort), '--rate', rate, loopFlag];

    const proc = spawn(py, args, { cwd: ROOT });
    proc.stdout.on('data', d => process.stdout.write(`[replay] ${d}`));
    proc.stderr.on('data', d => process.stderr.write(`[replay] ${d}`));
    proc.on('exit', (code, sig) => {
      console.log(`[replay] exit code=${code} sig=${sig}`);
      if (state.replay && state.replay.proc === proc) state.replay = null;
    });

    state.replay = { proc, file: name, startedAt: new Date().toISOString() };
    res.json({ started: true, file: name, pid: proc.pid });
  });

  router.post('/replay/stop', (req, res) => {
    if (!state.replay || state.replay.proc.killed) return res.status(409).json({ error: 'Not replaying' });
    state.replay.proc.kill('SIGINT');
    res.json({ stopped: true, file: state.replay.file });
  });

  return router;
}
