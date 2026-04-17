#!/usr/bin/env python3
"""
replay.py — Replay an Ouster PCAP to localhost UDP ports with original packet timing.

The Node.js lidar-proxy binds to 0.0.0.0:<lidar_port>, so sending packets to
127.0.0.1:<lidar_port> makes the server treat them identically to live traffic.
"""
import argparse
import socket
import sys
import time


def iter_raw_packets(pcap_path):
    """Yield (timestamp_sec, udp_dst_port, payload) tuples in capture order.

    We use dpkt-style parsing via ouster.sdk.pcap internal helper — fall back
    to scapy if available. Simpler: use scapy's rdpcap since it ships widely.
    """
    try:
        from scapy.all import rdpcap, UDP, Raw
    except ImportError:
        print("scapy required for replay. Install with: pip install scapy", file=sys.stderr)
        sys.exit(1)

    for pkt in rdpcap(pcap_path):
        if UDP in pkt and Raw in pkt:
            yield float(pkt.time), int(pkt[UDP].dport), bytes(pkt[Raw].load)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcap')
    ap.add_argument('--lidar-port', type=int, default=7502)
    ap.add_argument('--imu-port', type=int, default=7503)
    ap.add_argument('--dest', default='127.0.0.1')
    ap.add_argument('--rate', type=float, default=1.0, help='playback rate (1.0 = realtime)')
    ap.add_argument('--loop', dest='loop', action='store_true')
    ap.add_argument('--no-loop', dest='loop', action='store_false')
    ap.set_defaults(loop=False)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
        t0_wall = time.time()
        t0_pkt = None
        count = 0
        for ts, dport, payload in iter_raw_packets(args.pcap):
            if t0_pkt is None:
                t0_pkt = ts
            # Map original Ouster UDP ports to configured ports.
            # Ouster pcaps usually have dport = original sensor config
            # (e.g. 7502/7503). We keep relative: if dport is even-ish lidar-sized, send to lidar_port.
            if len(payload) >= 3000:
                out_port = args.lidar_port
            else:
                out_port = args.imu_port

            target = (t0_pkt is not None) and (t0_wall + (ts - t0_pkt) / max(args.rate, 1e-6))
            now = time.time()
            if target and target > now:
                time.sleep(target - now)
            sock.sendto(payload, (args.dest, out_port))
            count += 1
        print(f"[replay] sent {count} packets from {args.pcap}", flush=True)
        if not args.loop:
            break


if __name__ == '__main__':
    main()
