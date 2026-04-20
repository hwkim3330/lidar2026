/**
 * nav-api.js — Occupancy-grid mapping + A* planning + virtual ego tracker.
 *
 * Listens to live LiDAR frames, accumulates a 2D top-down occupancy grid
 * (log-odds style via hit/visit counters), supports save/load, answers
 * A* path plans from origin to clicked goal, and streams occupancy +
 * path updates over WebSocket.
 */

import { Router } from 'express';
import path from 'path';
import fs from 'fs';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');
const MAP_DIR = path.join(ROOT, 'data', 'nav_maps');

// Grid config
const CELL = 0.2;              // m per cell
const SIZE = 500;              // 500×500 cells = 100m × 100m
const HALF = SIZE / 2;          // origin at cell (HALF, HALF)
const MAX_COUNT = 250;          // cap per-cell counter
const HIT_WIN = 60;             // occupancy if hitRatio over this many visits

function gridIdx(cx, cy) { return cy * SIZE + cx; }
function worldToCell(x, y) {
  return [Math.round(x / CELL) + HALF, Math.round(y / CELL) + HALF];
}
function cellToWorld(cx, cy) {
  return [(cx - HALF) * CELL, (cy - HALF) * CELL];
}

export function createNav(lidarInstance, lidarConfig) {
  const state = {
    hit: new Uint16Array(SIZE * SIZE),    // occupied votes
    visit: new Uint16Array(SIZE * SIZE),  // total votes (hit or pass-through)
    lastUpdateT: 0,
    updates: 0,
    clients: new Set(),
    goal: null,                           // { x, y }
    path: null,                           // [[x,y], ...]
    ego: { x: 0, y: 0, yaw: 0, onPath: false, idx: 0, speed: 1.5 }, // virtual
    groundCutMm: -300,
    ceilCutMm: 2500,
    mapName: 'live',
  };

  // Expose hook: lidarInstance calls our tap on each broadcast
  function tapPointCloud(points) {
    // points: array of { x, y, z, intensity } in sensor frame (meters)
    const origin = worldToCell(0, 0);
    const gcut = state.groundCutMm / 1000;
    const ccut = state.ceilCutMm / 1000;

    for (const p of points) {
      if (p.z < gcut || p.z > ccut) continue;
      const d = Math.hypot(p.x, p.y);
      if (d < 0.3 || d > 49) continue;
      const end = worldToCell(p.x, p.y);
      // Bresenham from origin to end, mark visit along, hit at end
      raycast(origin[0], origin[1], end[0], end[1], (cx, cy, isEnd) => {
        if (cx < 0 || cy < 0 || cx >= SIZE || cy >= SIZE) return false;
        const idx = gridIdx(cx, cy);
        if (state.visit[idx] < MAX_COUNT) state.visit[idx]++;
        if (isEnd && state.hit[idx] < MAX_COUNT) state.hit[idx]++;
        return true;
      });
    }
    state.updates++;
    state.lastUpdateT = Date.now();
  }

  function raycast(x0, y0, x1, y1, cb) {
    // Bresenham's line. cb(x, y, isEnd) — stop if cb returns false.
    const dx = Math.abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    const dy = -Math.abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    let err = dx + dy, x = x0, y = y0;
    while (true) {
      const isEnd = (x === x1 && y === y1);
      if (cb(x, y, isEnd) === false) return;
      if (isEnd) return;
      const e2 = 2 * err;
      if (e2 >= dy) { err += dy; x += sx; }
      if (e2 <= dx) { err += dx; y += sy; }
    }
  }

  // Hook into lidar instance — monkey-patch its existing broadcastFrame?
  // Cleaner: add a callback via a new instance method. We'll add setOnFrame.
  lidarInstance.setOnFrame?.(tapPointCloud);

  // ── Occupancy classification ──
  function classify(idx) {
    const v = state.visit[idx], h = state.hit[idx];
    if (v < 3) return 'unknown';
    const ratio = h / v;
    if (ratio > 0.4) return 'occupied';
    if (v - h >= 3) return 'free';
    return 'unknown';
  }

  // ── A* path planner ──
  function plan(goalWorld) {
    const start = worldToCell(0, 0);
    const goal = worldToCell(goalWorld.x, goalWorld.y);
    if (goal[0] < 0 || goal[0] >= SIZE || goal[1] < 0 || goal[1] >= SIZE) return null;

    // Inflate obstacles by 1 cell to keep clearance
    const blocked = new Uint8Array(SIZE * SIZE);
    for (let i = 0; i < state.visit.length; i++) {
      if (classify(i) === 'occupied') blocked[i] = 1;
    }
    // Dilate once
    const dil = new Uint8Array(blocked);
    for (let y = 1; y < SIZE - 1; y++) {
      for (let x = 1; x < SIZE - 1; x++) {
        if (blocked[gridIdx(x, y)]) continue;
        if (blocked[gridIdx(x-1,y)] || blocked[gridIdx(x+1,y)] || blocked[gridIdx(x,y-1)] || blocked[gridIdx(x,y+1)]) {
          dil[gridIdx(x, y)] = 1;
        }
      }
    }

    // A*
    const h = (x, y) => Math.hypot(x - goal[0], y - goal[1]);
    const open = new Map();     // key → { f, g, x, y, parent }
    const closed = new Set();
    const startKey = `${start[0]},${start[1]}`;
    open.set(startKey, { f: h(start[0], start[1]), g: 0, x: start[0], y: start[1], parent: null });

    const neigh = [[1,0,1],[-1,0,1],[0,1,1],[0,-1,1],[1,1,1.41],[1,-1,1.41],[-1,1,1.41],[-1,-1,1.41]];
    let steps = 0;

    while (open.size) {
      // Find min-f in open (linear — fine for 500×500 moderate use)
      let bestK = null, bestN = null;
      for (const [k, n] of open) { if (!bestN || n.f < bestN.f) { bestN = n; bestK = k; } }
      open.delete(bestK); closed.add(bestK);

      if (bestN.x === goal[0] && bestN.y === goal[1]) {
        // Reconstruct
        const cells = [];
        let cur = bestN;
        while (cur) { cells.push([cur.x, cur.y]); cur = cur.parent; }
        cells.reverse();
        // Simplify / downsample (every 2nd cell)
        const ws = cells.filter((_, i) => i % 2 === 0 || i === cells.length - 1).map(([cx, cy]) => cellToWorld(cx, cy));
        return ws;
      }

      for (const [dx, dy, cost] of neigh) {
        const nx = bestN.x + dx, ny = bestN.y + dy;
        if (nx < 0 || ny < 0 || nx >= SIZE || ny >= SIZE) continue;
        if (dil[gridIdx(nx, ny)]) continue;
        const nk = `${nx},${ny}`;
        if (closed.has(nk)) continue;
        const g = bestN.g + cost;
        const existing = open.get(nk);
        if (existing && existing.g <= g) continue;
        open.set(nk, { f: g + h(nx, ny), g, x: nx, y: ny, parent: bestN });
      }
      steps++;
      if (steps > 200000) return null;
    }
    return null;
  }

  // ── Snapshot: serialize grid for save/stream ──
  function snapshot() {
    // Compress: only cells with visit > 0
    const out = [];
    for (let i = 0; i < state.visit.length; i++) {
      if (state.visit[i] > 0) {
        out.push([i, state.hit[i], state.visit[i]]);
      }
    }
    return {
      cell: CELL, size: SIZE, half: HALF,
      updates: state.updates,
      cells: out,
    };
  }

  function restore(snap) {
    state.hit.fill(0); state.visit.fill(0);
    for (const [i, h, v] of snap.cells) { state.hit[i] = h; state.visit[i] = v; }
    state.updates = snap.updates || 0;
    state.lastUpdateT = Date.now();
  }

  // WS streaming helper
  function streamSnapshot(ws) {
    const snap = snapshot();
    ws.send(JSON.stringify({ type: 'grid', ...snap }));
  }

  function broadcastPath() {
    for (const ws of state.clients) {
      if (ws.readyState !== 1) continue;
      ws.send(JSON.stringify({ type: 'path', goal: state.goal, path: state.path }));
    }
  }

  // ── Virtual ego tracker along path ──
  let egoTimer = null;
  function startFollow() {
    stopFollow();
    if (!state.path || state.path.length < 2) return;
    state.ego.idx = 0;
    state.ego.onPath = true;
    let lastT = performance.now();
    egoTimer = setInterval(() => {
      const now = performance.now();
      const dt = (now - lastT) / 1000;
      lastT = now;
      const wp = state.path[state.ego.idx + 1];
      if (!wp) { stopFollow(); return; }
      const dx = wp[0] - state.ego.x, dy = wp[1] - state.ego.y;
      const d = Math.hypot(dx, dy);
      if (d < 0.15) { state.ego.idx++; }
      else {
        state.ego.yaw = Math.atan2(dy, dx);
        const step = Math.min(d, state.ego.speed * dt);
        state.ego.x += Math.cos(state.ego.yaw) * step;
        state.ego.y += Math.sin(state.ego.yaw) * step;
      }
      for (const ws of state.clients) {
        if (ws.readyState !== 1) continue;
        ws.send(JSON.stringify({ type: 'ego', x: state.ego.x, y: state.ego.y, yaw: state.ego.yaw, idx: state.ego.idx, onPath: true }));
      }
    }, 50);
  }
  function stopFollow() {
    if (egoTimer) clearInterval(egoTimer);
    egoTimer = null;
    state.ego.onPath = false;
  }

  // ── Router ──
  const router = Router();

  router.get('/status', (req, res) => {
    let occupied = 0, free = 0, unknown = 0;
    for (let i = 0; i < state.visit.length; i++) {
      const c = classify(i);
      if (c === 'occupied') occupied++;
      else if (c === 'free') free++;
      else if (state.visit[i] > 0) unknown++;
    }
    res.json({
      size: SIZE, cell: CELL,
      updates: state.updates,
      cells: { occupied, free, unknown },
      goal: state.goal, hasPath: !!state.path,
      ego: state.ego,
      mapName: state.mapName,
    });
  });

  router.get('/snapshot', (req, res) => res.json(snapshot()));

  router.post('/clear', (req, res) => {
    state.hit.fill(0); state.visit.fill(0); state.updates = 0;
    state.goal = null; state.path = null; stopFollow();
    for (const ws of state.clients) if (ws.readyState === 1) ws.send(JSON.stringify({ type: 'clear' }));
    res.json({ ok: true });
  });

  router.post('/plan', (req, res) => {
    const { x, y } = req.body || {};
    if (typeof x !== 'number' || typeof y !== 'number') return res.status(400).json({ error: 'x,y required' });
    state.goal = { x, y };
    const t0 = performance.now();
    state.path = plan(state.goal);
    const elapsedMs = performance.now() - t0;
    broadcastPath();
    res.json({ goal: state.goal, path: state.path, elapsedMs: Math.round(elapsedMs), length: state.path?.length || 0 });
  });

  router.post('/follow/start', (req, res) => { state.ego.x = 0; state.ego.y = 0; startFollow(); res.json({ ok: true }); });
  router.post('/follow/stop', (req, res) => { stopFollow(); res.json({ ok: true }); });

  router.post('/save', (req, res) => {
    const name = (req.body?.name || `map-${new Date().toISOString().replace(/[:.]/g,'-').slice(0,19)}`).replace(/[^\w.-]/g,'');
    fs.mkdirSync(MAP_DIR, { recursive: true });
    const fpath = path.join(MAP_DIR, name + '.json');
    const snap = snapshot();
    fs.writeFileSync(fpath, JSON.stringify(snap));
    state.mapName = name;
    res.json({ saved: name, cells: snap.cells.length, path: fpath });
  });

  router.get('/maps', (req, res) => {
    fs.mkdirSync(MAP_DIR, { recursive: true });
    const files = fs.readdirSync(MAP_DIR).filter(f => f.endsWith('.json')).map(f => {
      const st = fs.statSync(path.join(MAP_DIR, f));
      return { name: f.replace(/\.json$/, ''), sizeKB: Math.round(st.size / 1024), mtime: st.mtime.toISOString() };
    }).sort((a, b) => b.mtime.localeCompare(a.mtime));
    res.json(files);
  });

  router.post('/load', (req, res) => {
    const name = String(req.body?.name || '').replace(/[^\w.-]/g, '');
    const fpath = path.join(MAP_DIR, name + '.json');
    if (!fs.existsSync(fpath)) return res.status(404).json({ error: 'not found' });
    const snap = JSON.parse(fs.readFileSync(fpath, 'utf8'));
    restore(snap);
    state.mapName = name;
    for (const ws of state.clients) if (ws.readyState === 1) streamSnapshot(ws);
    res.json({ loaded: name, cells: snap.cells.length });
  });

  router.delete('/maps/:name', (req, res) => {
    const n = req.params.name.replace(/[^\w.-]/g, '');
    const fpath = path.join(MAP_DIR, n + '.json');
    if (fs.existsSync(fpath)) fs.unlinkSync(fpath);
    res.json({ deleted: n });
  });

  // ── WebSocket handler (to be attached in server.js) ──
  function handleWS(ws) {
    state.clients.add(ws);
    streamSnapshot(ws);
    ws.send(JSON.stringify({ type: 'path', goal: state.goal, path: state.path }));
    const iv = setInterval(() => { if (ws.readyState === 1) streamSnapshot(ws); }, 2000);
    ws.on('close', () => { state.clients.delete(ws); clearInterval(iv); });
  }

  return { router, handleWS, state, snapshot };
}
