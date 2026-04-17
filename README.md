# lidar2026

Ouster OS-1-16 **실시간 시각화 + 주기/지터 분석 + PCAP 녹화/재생** 웹 플랫폼.

USB 랜카드에 직결된 Ouster 센서 하나로 동작하며, Node.js 서버가 UDP 패킷을 실시간 수신해
브라우저(Three.js)로 3D 포인트클라우드를 스트리밍하고, 동시에 **나노초 정밀**으로 패킷 타이밍을
측정해 주기/지터 그래프를 표시합니다. 모든 녹화본은 **표준 libpcap (DLT_EN10MB)** 형식으로
저장되므로 Wireshark, `ouster-cli`, 그리고 이 레포의 재생 기능으로 바로 사용할 수 있으며
Hugging Face / AI Hub 업로드에 적합합니다.

```
Ouster OS-1-16 ── UDP:7502 ──▶ Node (lidar-proxy) ─┬─▶ WebSocket ──▶ Browser (Three.js)
                                                    ├─▶ PcapWriter ──▶ data/recordings/*.pcap
                                                    └─▶ Profiler ──▶ /api/lidar/profile
                                                                      (period, jitter, FPS, BW)

data/recordings/*.pcap  ──▶ scripts/replay.py ──▶ UDP 127.0.0.1:7502 ──▶ (same viz pipeline)
```

## Quick Start

```bash
git clone https://github.com/hwkim3330/lidar2026.git
cd lidar2026
bash run.sh          # venv + npm install 후 http://localhost:3000
```

네트워크 (Ubuntu + NetworkManager, USB NIC = `enxc84d44263ba6`):

```bash
sudo nmcli con add type ethernet ifname enxc84d44263ba6 con-name lidar-usb \
    ipv4.method shared ipv4.addresses 192.168.1.1/24 ipv6.method disabled
sudo nmcli con up lidar-usb
# Ouster가 DHCP 로 192.168.1.210 할당받음 (dnsmasq lease 확인)

curl -X POST http://192.168.1.210/api/v1/sensor/config \
  -H 'Content-Type: application/json' \
  -d '{"udp_dest":"192.168.1.1","udp_port_lidar":7502,"udp_port_imu":7503}'
```

## Pages

- `/` — landing
- `/lidar-live.html` — 3D Live Point Cloud (Three.js, WebSocket)
- `/lidar-timing.html` — Packet interval / jitter histogram / FPS / bandwidth
- `/record.html` — PCAP Record / Replay control + file manager

## API

| Endpoint | Method | Body / Params | Purpose |
|---|---|---|---|
| `/api/config` | GET | — | lidar list |
| `/api/lidar/stats` | GET | — | live packet stats |
| `/api/lidar/stats/:id` | GET | — | per-lidar stats |
| `/api/lidar/profile/:id` | GET | — | period / jitter / FPS / BW |
| `/api/lidar/capture/:id` | POST | — | save timing snapshot JSON |
| `/api/lidar/captures` | GET | — | list timing JSONs |
| `/api/record/start` | POST | `{name?, lidarId?}` | begin PCAP + metadata |
| `/api/record/stop` | POST | — | finalize pcap |
| `/api/recordings` | GET | — | list pcaps |
| `/api/recordings/:name` | DELETE | — | delete pcap + sidecar |
| `/api/replay/start` | POST | `{file, rate?, loop?, lidarId?}` | replay pcap → `127.0.0.1:<udpPort>` |
| `/api/replay/stop` | POST | — | stop replay |
| `/api/status` | GET | — | current record / replay state |

## Recording format

- **`<name>.pcap`** — libpcap (magic `a1b2c3d4`, DLT_EN10MB). Each record = fake Ethernet
  + IPv4 + UDP header + raw Ouster payload. Readable by Wireshark, `tcpreplay`, scapy,
  `ouster-cli source <file> viz`.
- **`<name>.json`** — Ouster sensor metadata fetched at record start from
  `http://<sensor>/api/v1/sensor/metadata` (beam intrinsics, lidar_mode, column counts, ...).
  Required by Ouster SDK for playback.

Hugging Face / AI Hub 업로드 시 pcap + json 쌍을 그대로 업로드하면 됩니다.

## Project layout

```
server/
  server.js         Express + WebSocket bootstrap
  lidar-proxy.js    UDP → WS bridge, frame parser, timing profiler
  pcap-writer.js    Native libpcap writer (DLT_EN10MB)
  record-api.js     Record / replay / list / delete endpoints
  config.js         Sensor IP / ports / paths
scripts/
  replay.py         PCAP → UDP replay (scapy, realtime-paced)
lidar-live.html     3D viewer
lidar-timing.html   timing graphs
record.html         record / replay UI
.venv/              Python SDK env (ouster-sdk + scapy)
data/recordings/    *.pcap + *.json
```

## External viewers

```bash
# Native Ouster visualizer (OpenGL window)
.venv/bin/ouster-cli source data/recordings/<file>.pcap viz

# Convert to OSF
.venv/bin/ouster-cli source data/recordings/<file>.pcap -m data/recordings/<file>.json save out.osf

# Inspect with Wireshark
wireshark data/recordings/<file>.pcap
```
