/**
 * sensor-api.js — Proxy to Ouster HTTP API for sensor config (lidar_mode etc.)
 */
import { Router } from 'express';
import { lidars } from './config.js';

export default function sensorApi() {
  const router = Router();
  const host = lidars[0].host;

  const VALID_MODES = ['512x10', '512x20', '1024x10', '1024x20', '2048x10'];

  router.get('/config', async (req, res) => {
    try {
      const r = await fetch(`http://${host}/api/v1/sensor/config`, { signal: AbortSignal.timeout(4000) });
      const cfg = await r.json();
      res.json({ host, config: cfg });
    } catch (e) { res.status(502).json({ error: e.message }); }
  });

  router.post('/mode', async (req, res) => {
    const mode = String(req.body?.mode || '');
    if (!VALID_MODES.includes(mode)) return res.status(400).json({ error: `invalid mode. allowed: ${VALID_MODES.join(', ')}` });
    try {
      const r = await fetch(`http://${host}/api/v1/sensor/config`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ lidar_mode: mode }),
        signal: AbortSignal.timeout(8000),
      });
      const txt = await r.text();
      res.json({ ok: r.ok, status: r.status, body: txt, mode, note: 'sensor will reinitialize — expect ~10s outage' });
    } catch (e) { res.status(502).json({ error: e.message }); }
  });

  router.post('/restart', async (req, res) => {
    try {
      const r = await fetch(`http://${host}/api/v1/sensor/cmd/restart`, { method: 'POST', signal: AbortSignal.timeout(8000) });
      const txt = await r.text();
      res.json({ ok: r.ok, status: r.status, body: txt });
    } catch (e) { res.status(502).json({ error: e.message }); }
  });

  router.get('/metadata', async (req, res) => {
    try {
      const r = await fetch(`http://${host}/api/v1/sensor/metadata`, { signal: AbortSignal.timeout(5000) });
      res.json(await r.json());
    } catch (e) { res.status(502).json({ error: e.message }); }
  });

  // Convenience aggregator: pulls everything we can in parallel
  router.get('/all', async (req, res) => {
    const eps = ['sensor/config', 'sensor/metadata', 'system/firmware', 'sensor/alerts', 'time', 'system/network'];
    const out = {};
    await Promise.all(eps.map(async (ep) => {
      try {
        const r = await fetch(`http://${host}/api/v1/${ep}`, { signal: AbortSignal.timeout(4000) });
        if (r.ok) out[ep.replace('/', '_')] = await r.json();
        else out[ep.replace('/', '_')] = { _http_status: r.status };
      } catch (e) { out[ep.replace('/', '_')] = { _error: e.message }; }
    }));
    res.json({ host, ts: Date.now(), endpoints: out });
  });

  // Generic proxy: GET /api/sensor/proxy?path=<path-after-/api/v1/>
  router.get('/proxy', async (req, res) => {
    const p = String(req.query.path || '').replace(/^\/+/, '');
    if (!p) return res.status(400).json({ error: 'path required' });
    if (!/^[a-z0-9_\/-]+$/i.test(p)) return res.status(400).json({ error: 'invalid path' });
    try {
      const r = await fetch(`http://${host}/api/v1/${p}`, { signal: AbortSignal.timeout(5000) });
      const txt = await r.text();
      let body;
      try { body = JSON.parse(txt); } catch { body = txt; }
      res.json({ path: p, status: r.status, ok: r.ok, body });
    } catch (e) { res.status(502).json({ error: e.message }); }
  });

  // POST proxy
  router.post('/proxy', async (req, res) => {
    const p = String(req.body?.path || '').replace(/^\/+/, '');
    if (!p || !/^[a-z0-9_\/-]+$/i.test(p)) return res.status(400).json({ error: 'invalid path' });
    try {
      const r = await fetch(`http://${host}/api/v1/${p}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(req.body?.body ?? {}),
        signal: AbortSignal.timeout(8000),
      });
      const txt = await r.text();
      let body;
      try { body = JSON.parse(txt); } catch { body = txt; }
      res.json({ path: p, status: r.status, ok: r.ok, body });
    } catch (e) { res.status(502).json({ error: e.message }); }
  });

  return router;
}
