#!/usr/bin/env python3
"""
map_pcap.py — Build a 3D map from an Ouster PCAP using KISS-ICP.

Outputs:
  <out>.ply     : voxel-downsampled accumulated point cloud (world frame)
  <out>.json    : summary (scans, map points, bounds, elapsed)

Progress (one JSON line per scan) goes to stdout:
  {"scan": i, "total": N, "pose": [tx,ty,tz], "map_points": K}
"""
import argparse
import json
import sys
import time

import numpy as np

from ouster.sdk import open_source
from ouster.sdk.core import XYZLut
from kiss_icp.kiss_icp import KissICP
from kiss_icp.config import KISSConfig


def write_ply(path, points):
    with open(path, "wb") as f:
        header = (
            "ply\n"
            "format binary_little_endian 1.0\n"
            f"element vertex {len(points)}\n"
            "property float x\nproperty float y\nproperty float z\n"
            "end_header\n"
        ).encode("ascii")
        f.write(header)
        f.write(points.astype(np.float32).tobytes())


def emit(event):
    sys.stdout.write(json.dumps(event) + "\n")
    sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcap")
    ap.add_argument("meta")
    ap.add_argument("out", help="output basename (no extension)")
    ap.add_argument("--voxel", type=float, default=0.1, help="map voxel size (m)")
    ap.add_argument("--max-range", type=float, default=40.0)
    ap.add_argument("--min-range", type=float, default=0.5)
    ap.add_argument("--max-scans", type=int, default=0, help="0 = all")
    args = ap.parse_args()

    t0 = time.time()
    src = open_source(args.pcap, meta=[args.meta])
    info = src.sensor_info[0]
    lut = XYZLut(info)

    cfg = KISSConfig()
    cfg.data.max_range = args.max_range
    cfg.data.min_range = args.min_range
    cfg.data.deskew = False
    cfg.mapping.voxel_size = args.voxel
    odom = KissICP(cfg)

    # Map stored as voxel set → world-frame centroid
    voxel_map = {}
    V = args.voxel

    def add_to_map(pts_w):
        keys = np.floor(pts_w / V).astype(np.int64)
        for k, p in zip(map(tuple, keys), pts_w):
            if k not in voxel_map:
                voxel_map[k] = p

    emit({"event": "start", "pcap": args.pcap, "voxel": V})

    i = 0
    for scanset in src:
        scan_iter = list(scanset.valid_scans())
        if not scan_iter:
            i += 1; continue
        scan = scan_iter[0]
        xyz_hwc = lut(scan)
        r2 = xyz_hwc[:, :, 0] ** 2 + xyz_hwc[:, :, 1] ** 2 + xyz_hwc[:, :, 2] ** 2
        mask = (r2 > args.min_range ** 2) & (r2 < args.max_range ** 2)
        xyz = xyz_hwc[mask]
        if xyz.shape[0] < 100:
            i += 1
            continue
        timestamps = np.zeros(len(xyz), dtype=np.float64)  # no deskew
        try:
            odom.register_frame(xyz, timestamps)
        except Exception as e:
            emit({"event": "warn", "scan": i, "err": str(e)})
            i += 1
            continue

        pose = odom.last_pose  # 4x4 numpy
        xyz_h = np.hstack([xyz, np.ones((xyz.shape[0], 1))])
        xyz_w = (xyz_h @ pose.T)[:, :3]
        add_to_map(xyz_w)

        t = pose[:3, 3]
        if i % 5 == 0:
            emit({"event": "scan", "i": i, "pose": [float(t[0]), float(t[1]), float(t[2])], "map_points": len(voxel_map)})

        i += 1
        if args.max_scans and i >= args.max_scans:
            break

    if not voxel_map:
        emit({"event": "error", "msg": "no map points"})
        sys.exit(1)

    map_points = np.array(list(voxel_map.values()), dtype=np.float32)
    ply_path = args.out + ".ply"
    write_ply(ply_path, map_points)

    summary = {
        "event": "done",
        "scans": i,
        "map_points": int(len(map_points)),
        "bounds_min": map_points.min(axis=0).tolist(),
        "bounds_max": map_points.max(axis=0).tolist(),
        "elapsed_sec": round(time.time() - t0, 2),
        "ply": ply_path,
    }
    with open(args.out + ".json", "w") as f:
        json.dump(summary, f, indent=2)
    emit(summary)


if __name__ == "__main__":
    main()
