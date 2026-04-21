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

  return router;
}
