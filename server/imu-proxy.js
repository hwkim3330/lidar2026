/**
 * imu-proxy.js — Ouster IMU UDP → WebSocket bridge.
 *
 * Ouster IMU (LEGACY profile) packet = 48 bytes @ ~100 Hz:
 *   [0..8)   system_timestamp (uint64 ns)
 *   [8..16)  accel_timestamp  (uint64 ns)
 *   [16..24) gyro_timestamp   (uint64 ns)
 *   [24..36) accel x/y/z      (float32, g-force; +Z up)
 *   [36..48) gyro  x/y/z      (float32, deg/s)
 */

import dgram from 'dgram';

export function setupImuProxy(wss, lidars) {
  const cfg = lidars[0];
  const port = cfg.imuPort ?? 7503;
  const udp = dgram.createSocket({ type: 'udp4', reuseAddr: true });

  const clients = new Set();
  const wsPath = (cfg.wsPath || '/ws/lidar-a').replace('/ws/lidar', '/ws/imu');

  wss.on('connection', (ws, req) => {
    if (req.url === wsPath) {
      clients.add(ws);
      ws.on('close', () => clients.delete(ws));
    }
  });

  const stats = { pktCount: 0, lastT: 0 };

  udp.on('message', (msg) => {
    if (msg.length !== 48) return;
    stats.pktCount++;
    const sysTs = Number(msg.readBigUInt64LE(0));
    const accTs = Number(msg.readBigUInt64LE(8));
    const gyrTs = Number(msg.readBigUInt64LE(16));
    const ax = msg.readFloatLE(24);
    const ay = msg.readFloatLE(28);
    const az = msg.readFloatLE(32);
    const gx = msg.readFloatLE(36);
    const gy = msg.readFloatLE(40);
    const gz = msg.readFloatLE(44);

    const nowUs = Number(process.hrtime.bigint()) / 1000;
    stats.lastT = nowUs;

    if (clients.size === 0) return;
    const m = JSON.stringify({
      type: 'imu',
      t: Math.round(nowUs),
      sysTs, accTs, gyrTs,
      ax, ay, az, gx, gy, gz,
    });
    for (const ws of clients) if (ws.readyState === 1) ws.send(m);
  });

  udp.on('error', (e) => console.error(`  IMU UDP error:`, e.message));
  udp.bind(port, '0.0.0.0', () => console.log(`  IMU: UDP :${port} → WebSocket ${wsPath}`));

  return { stats };
}
