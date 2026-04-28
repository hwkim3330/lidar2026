/**
 * motor-api.js — Virtual motor state for the FSD page.
 * Tracks commanded throttle / brake / gear, computes a kinematic
 * virtual speed, exposes state. No real actuator (laptop USB has no
 * per-port power switching anyway). Audible feedback is done client-side
 * via Web Audio.
 */

import { Router } from 'express';
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');
const LOG_PATH = path.join(ROOT, 'data', 'motor.log');

export default function motorApi() {
  const state = {
    throttle: 0, brake: 0, gear: 'P',
    autopilot: false, eStop: false,
    rpm: 0, speedKmh: 0, accelMps2: 0,
    cmdT: 0, history: [],
  };

  // 50 Hz physics tick
  let lastT = Date.now();
  setInterval(() => {
    const now = Date.now();
    const dt = (now - lastT) / 1000;
    lastT = now;
    if (state.eStop) { state.throttle = 0; state.brake = 1; }

    const dir = state.gear === 'R' ? -1 : (state.gear === 'D' ? 1 : 0);
    // Simple longitudinal model (no actual mass): m=1, drag=0.4, gain=12, brake=10
    const accel = dir * state.throttle * 12 - state.brake * 10 - 0.4 * (state.speedKmh / 3.6);
    state.accelMps2 = accel;
    const v = state.speedKmh / 3.6 + accel * dt;
    state.speedKmh = Math.max(0, v) * 3.6;
    state.rpm = Math.round(state.throttle * 9000);
  }, 20);

  function applyCmd(throttle, brake, gear) {
    if (typeof throttle === 'number') state.throttle = Math.max(0, Math.min(1, throttle));
    if (typeof brake === 'number') state.brake = Math.max(0, Math.min(1, brake));
    if (gear) state.gear = gear;
    state.cmdT = Date.now();
    state.history.push({ t: state.cmdT, throttle: state.throttle, brake: state.brake, gear: state.gear });
    if (state.history.length > 200) state.history.shift();
    try {
      fs.mkdirSync(path.dirname(LOG_PATH), { recursive: true });
      fs.appendFileSync(LOG_PATH, JSON.stringify({ t: state.cmdT, throttle: state.throttle, brake: state.brake, gear: state.gear, speedKmh: Math.round(state.speedKmh * 10) / 10 }) + '\n');
    } catch {}
  }

  const snapshot = () => ({ ...state, history: undefined });

  const router = Router();
  router.get('/state', (req, res) => res.json(snapshot()));
  router.get('/history', (req, res) => res.json(state.history.slice(-100)));
  router.post('/cmd', (req, res) => {
    if (state.eStop && req.body?.releaseEstop !== true) {
      return res.status(409).json({ error: 'E-STOP engaged' });
    }
    applyCmd(req.body?.throttle, req.body?.brake, req.body?.gear);
    res.json(snapshot());
  });
  router.post('/autopilot', (req, res) => { state.autopilot = !!req.body?.enabled; res.json(snapshot()); });
  router.post('/estop', (req, res) => {
    if (req.body?.release === true) { state.eStop = false; res.json({ ...snapshot(), released: true }); return; }
    state.eStop = true; state.throttle = 0; state.brake = 1;
    res.json({ ...snapshot(), engaged: true });
  });
  return router;
}
