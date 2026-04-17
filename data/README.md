# LiDAR Timing Capture Data

Ouster OS-1-16 실측 패킷 타이밍 데이터.

## Profiling Pipeline

```
LiDAR UDP packets
    │
    ▼
Server (lidar-proxy.js)
    ├── process.hrtime.bigint() — 나노초 타임스탬프 기록
    ├── 슬라이딩 윈도우: 500 packets + 100 frames (≈10초)
    ├── 최소 5프레임(0.5초) 후 프로파일 생성 가능
    ├── 100프레임(10초) 후 통계 안정화
    └── 1초마다 브라우저에 profile + auto-TAS 전송
            │
            ▼
       Browser (lidar-timing.html)
            ├── Oscilloscope — 패킷 도착 시간 시각화
            ├── Frame Detail — 32패킷 상세
            ├── TAS Overlay — 자동 생성된 게이트 스케줄
            ├── Jitter Histogram — 패킷 간격 분포
            └── "Push to Board" → LAN9662 TAS 적용
```

## Timing Breakdown

| 단계 | 시간 | 설명 |
|------|------|------|
| 서버 시작 → 첫 패킷 | 즉시 | UDP 소켓 바인드 |
| 프로파일 최소 데이터 | **0.5초** (5프레임) | `frameTimestamps.length >= 3` |
| 안정된 통계 | **10초** (100프레임) | 슬라이딩 윈도우 full |
| 브라우저 갱신 주기 | **1초** | 10프레임마다 WS push |
| Push to Board | **0.2초** (실측 196ms) | CoAP iPATCH via Ethernet |
| Profile 계산 | **0.02 ms** | getTrafficProfile() |
| TAS 생성 | **0.02 ms** | generateTasConfig() |

**결론: 프로파일 안정화 10초 + 보드 Push 0.2초 = TAS 적용 완료**

### End-to-End Benchmark (2026-04-10, Ethernet)

```
Profile:    0.02 ms   ← 메모리 내 계산 (100프레임 통계)
TAS gen:    0.02 ms   ← 패킷 간격 → 사이클 → 게이트 엔트리
Board push: 196 ms    ← CoAP iPATCH, keti-tsn-cli, LAN9662 port '1'
Total:      197 ms    ← 프로파일 안정화 이후 처리 시간
```

- **Board port names**: `'1'`, `'2'` (숫자 문자열, NOT swp0/swp1)
- **Board boot time**: 하드 리셋 후 YANG 데이터스토어 초기화 ~8분 소요
- **no-sec 필수**: CoAP no-sec 패치 + save-config + reboot 후 이더넷 iPATCH 가능

## Capture API

```bash
# 현재 타이밍 스냅샷 저장
curl -X POST http://localhost:3000/api/lidar/capture/lidar-a

# 저장된 캡처 목록
curl http://localhost:3000/api/lidar/captures

# 특정 캡처 조회
curl http://localhost:3000/api/lidar/captures/lidar-capture-2026-04-10T00-47-53.json

# 실시간 프로파일
curl http://localhost:3000/api/lidar/profile/lidar-a

# 자동 TAS 설정
curl http://localhost:3000/api/lidar/auto-tas/lidar-a
```

## Capture File Format

```jsonc
{
  "capturedAt": "2026-04-10T00:47:53.589Z",
  "sensor": "Ouster OS-1-16 Gen1",
  "lidarId": "lidar-a",

  // 프레임 단위 통계 (100프레임 윈도우)
  "profile": {
    "fps": 10,                    // 프레임 레이트
    "frameIntervalUs": 100008,    // 평균 프레임 간격
    "jitterUs": 197,              // 프레임 지터 (σ)
    "pktsPerFrame": 32,           // 프레임당 패킷 수
    "pktSize": 3392,              // 패킷 크기 (bytes)
    "pktIntervalUs": 3125,        // 평균 패킷 간격
    "bandwidthMbps": 8.68         // 대역폭
  },

  // 자동 도출된 TAS 설정
  "autoTas": {
    "cycleUs": 3130,              // 패킷 간격에서 도출 (10µs 단위 반올림)
    "pktTxUs": 27.44,             // 와이어 타임: (3392+38)×8/1000
    "entries": [
      { "gateStates": 128, "durationUs": 54.9 },   // TC7 LiDAR (×2 마진)
      { "gateStates": 0,   "durationUs": 1 },       // guard
      { "gateStates": 127, "durationUs": 3073.1 },  // TC0-6 BE
      { "gateStates": 0,   "durationUs": 1 }        // guard
    ],
    "utilization": 1.75           // TC7 점유율 (%)
  },

  // 원시 타이밍 데이터 (500 packets)
  "timing": {
    "count": 500,
    "timestamps_us": [0, 3125.2, 6250.1, ...],    // 상대 시간
    "intervals_us": [3125.2, 3124.9, ...],          // 패킷 간격
    "frameIntervals_us": [100008, 99998, ...]       // 프레임 간격
  }
}
```

## Measured Values (2026-04-10)

| Metric | Value | Note |
|--------|-------|------|
| Packet size | 3,392 B | 16 cols × 212 B (LEGACY format) |
| Packets/frame | 32 | 512 cols / 16 cols per pkt |
| Frame rate | 10 Hz | = 100,000 µs period |
| Packet interval | 3,125 µs | = 100,000 / 32 |
| Packet jitter (σ) | 103 µs | |
| Frame jitter (σ) | 197 µs | min 99,548 ~ max 100,530 µs |
| Bandwidth | 8.68 Mbps | 3,392 × 32 × 10 × 8 / 1e6 |
| Wire time | 27.44 µs | (3,392 + 38) × 8 / 1,000 Mbps |
| TAS cycle | 3,125 µs | = 패킷 간격 (반올림 없음, ±5µs lock) |
| TC7 slot | 54.9 µs | wire time × 2x margin |
| TC7 utilization | 1.76% | 54.9 / 3,125 |
