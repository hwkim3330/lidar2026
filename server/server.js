/**
 * server.js — LiDAR live visualization + timing + record/replay
 */

import express from 'express';
import { createServer } from 'http';
import { WebSocketServer } from 'ws';
import path from 'path';
import fs from 'fs';
import { fileURLToPath } from 'url';
import { setupLidarProxy } from './lidar-proxy.js';
import { lidars, defaultLidarWsPath } from './config.js';
import recordApi from './record-api.js';
import { startKernelRx } from './kernel-rx.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');

const app = express();
app.use(express.json({ limit: '5mb' }));
app.use(express.static(ROOT));
app.get('/', (req, res) => res.sendFile(path.join(ROOT, 'index.html')));

const port = parseInt(process.argv.find((_, i, a) => a[i - 1] === '--port') || process.env.PORT || '3000', 10);

const httpServer = createServer(app);
const wss = new WebSocketServer({ server: httpServer });
const lidar = setupLidarProxy(httpServer, wss, lidars, defaultLidarWsPath);

// Kernel-level rx timestamp (libpcap SO_TIMESTAMP) — immune to Node event loop stalls.
// Requires: sudo setcap cap_net_raw,cap_net_admin=eip /usr/bin/tcpdump
const KRX_IFACE = process.env.LIDAR_IFACE || 'enxc84d44263ba6';
let krx = null;
try {
  krx = startKernelRx({ iface: KRX_IFACE, port: lidars[0].udpPort });
  console.log(`  Kernel-rx: tcpdump on ${KRX_IFACE} (libpcap timestamps)`);
  // Inject krx stats into the per-frame profile broadcast so the timing UI sees
  // both node-loop rx and kernel rx jitter side-by-side.
  lidar.instances[0]?.setProfileDecorator?.((p) => Object.assign(p, krx.getStats()));
} catch (e) {
  console.warn(`  Kernel-rx: disabled (${e.message}) — set LIDAR_IFACE or install tcpdump cap`);
}

app.get('/api/config', (req, res) => res.json({ lidars, boards: [] }));
// Back-compat stubs so the existing live page doesn't crash
app.get('/api/boards', (req, res) => res.json([]));
app.get('/api/boards/status', (req, res) => res.json([]));
app.get('/api/lidar/stats', (req, res) => res.json(lidar.getStats()));

app.get('/api/lidar/stats/:id', (req, res) => {
  const inst = lidar.instances.find(i => i.id === req.params.id);
  if (!inst) return res.status(404).json({ error: 'LiDAR not found' });
  res.json(inst.getStats());
});

app.get('/api/lidar/profile/:id', (req, res) => {
  const inst = lidar.instances.find(i => i.id === req.params.id);
  if (!inst) return res.status(404).json({ error: 'LiDAR not found' });
  const profile = inst.getTrafficProfile();
  if (!profile) return res.json({ error: 'Not enough data yet' });
  if (krx) Object.assign(profile, krx.getStats());
  res.json(profile);
});

app.post('/api/lidar/capture/:id', (req, res) => {
  const inst = lidar.instances.find(i => i.id === req.params.id);
  if (!inst) return res.status(404).json({ error: 'LiDAR not found' });
  const profile = inst.getTrafficProfile();
  const timing = inst.getTimingSnapshot();
  if (!profile || !timing) return res.json({ error: 'Not enough data yet' });
  const capture = {
    capturedAt: new Date().toISOString(),
    sensor: 'Ouster OS-1-16 Gen1',
    lidarId: req.params.id,
    profile, timing,
  };
  const dataDir = path.join(ROOT, 'data');
  if (!fs.existsSync(dataDir)) fs.mkdirSync(dataDir, { recursive: true });
  const fname = `timing-${new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)}.json`;
  fs.writeFileSync(path.join(dataDir, fname), JSON.stringify(capture, null, 2));
  res.json({ saved: fname, profile, packets: timing.count });
});

app.get('/api/lidar/captures', (req, res) => {
  const dataDir = path.join(ROOT, 'data');
  if (!fs.existsSync(dataDir)) return res.json([]);
  res.json(fs.readdirSync(dataDir).filter(f => f.endsWith('.json')));
});

app.get('/api/lidar/captures/:file', (req, res) => {
  const fpath = path.join(ROOT, 'data', req.params.file);
  if (!fs.existsSync(fpath)) return res.status(404).json({ error: 'Not found' });
  res.json(JSON.parse(fs.readFileSync(fpath, 'utf8')));
});

app.use('/api', recordApi(lidar));

httpServer.listen(port, () => {
  console.log(`\n  lidar2026 — Ouster LiDAR viz + record/replay`);
  console.log(`  ────────────────────────────────────────────`);
  console.log(`  Local:   http://localhost:${port}`);
  console.log(`  Live:    http://localhost:${port}/lidar-live.html`);
  console.log(`  Timing:  http://localhost:${port}/lidar-timing.html`);
  console.log();
  lidars.forEach(l => console.log(`  LiDAR ${l.id}: ${l.host} → UDP :${l.udpPort} → ${l.wsPath}`));
  console.log();
});
