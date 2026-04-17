/**
 * config.js — LiDAR-only configuration for lidar2026
 *
 * Direct USB-NIC ↔ Ouster connection (no switch).
 * Host NIC should be 192.168.1.1/24 with NM shared mode (DHCP for sensor).
 */

export const lidars = [
  {
    id: 'lidar-a',
    host: '192.168.1.210',
    udpPort: 7502,
    imuPort: 7503,
    wsPath: '/ws/lidar-a',
    label: 'Ouster OS-1-16',
  },
];

export const defaultLidarWsPath = '/ws/lidar';

export const paths = {
  ousterCli: process.env.OUSTER_CLI || '.venv/bin/ouster-cli',
  python: process.env.PYTHON || '.venv/bin/python',
  dataDir: 'data',
  recordingsDir: 'data/recordings',
};
