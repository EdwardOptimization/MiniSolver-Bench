#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import requests

ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Import a short TUM RGB-D groundtruth segment to scenario format.")
    parser.add_argument("--name", required=True)
    parser.add_argument("--url", required=True)
    parser.add_argument("--start-time", type=float, required=True)
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--N", type=int, default=50)
    parser.add_argument("--output-root", type=Path, default=ROOT / "scenarios")
    return parser.parse_args()


def fetch_groundtruth(url: str) -> np.ndarray:
    resp = requests.get(url, timeout=120)
    resp.raise_for_status()
    lines = resp.text.splitlines()
    data = []
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 8:
            continue
        ts, tx, ty, tz, qx, qy, qz, qw = parts[:8]
        data.append((float(ts), float(tx), float(ty), float(tz), float(qx), float(qy), float(qz), float(qw)))
    if not data:
        raise RuntimeError("no valid rows in groundtruth file")
    return np.array(data)


def compute_velocity(t: np.ndarray, p: np.ndarray) -> np.ndarray:
    if p.size < 2:
        return np.zeros_like(p)
    dt = np.diff(t)
    valid = np.abs(dt) > 1e-12
    if not np.all(valid):
        # remove duplicate timestamps if any
        t, unique_idx = np.unique(t, return_index=True)
        p = p[unique_idx]
        dt = np.diff(t)
        valid = np.abs(dt) > 1e-12
    if p.size < 2:
        return np.zeros_like(p)

    v = np.zeros_like(p)
    if p.size >= 2 and np.any(valid):
        v[0] = (p[1] - p[0]) / dt[0] if valid[0] else 0.0
        for i in range(1, p.size - 1):
            if valid[i] and valid[i - 1]:
                v[i] = (p[i + 1] - p[i - 1]) / (t[i + 1] - t[i - 1])
            elif valid[i]:
                v[i] = (p[i + 1] - p[i]) / dt[i]
            elif valid[i - 1]:
                v[i] = (p[i] - p[i - 1]) / dt[i - 1]
            else:
                v[i] = 0.0
        if valid[-1]:
            v[-1] = (p[-1] - p[-2]) / dt[-1]
    return v


def write_scenario(args: argparse.Namespace, times: np.ndarray, xyz: np.ndarray, vxyz: np.ndarray) -> Path:
    scenario_dir = args.output_root / args.name
    scenario_dir.mkdir(parents=True, exist_ok=True)
    ref_csv = scenario_dir / "ref.csv"

    t_rel = times - times[0]
    ref_df = {
        "t": [f"{t:.17g}" for t in t_rel],
        "x_ref": [f"{x:.17g}" for x in xyz[:, 0]],
        "y_ref": [f"{y:.17g}" for y in xyz[:, 1]],
        "z_ref": [f"{z:.17g}" for z in xyz[:, 2]],
        "vx_ref": [f"{v:.17g}" for v in vxyz[:, 0]],
        "vy_ref": [f"{v:.17g}" for v in vxyz[:, 1]],
        "vz_ref": [f"{v:.17g}" for v in vxyz[:, 2]],
    }
    import pandas as pd
    pd.DataFrame(ref_df).to_csv(ref_csv, index=False)

    scenario = {
        "name": args.name,
        "domain": "robotics",
        "model": "double_integrator_3d_v1",
        "dt": args.dt,
        "N": args.N,
        "x0": [
            float(xyz[0, 0]),
            float(xyz[0, 1]),
            float(xyz[0, 2]),
            float(vxyz[0, 0]),
            float(vxyz[0, 1]),
            float(vxyz[0, 2]),
        ],
        "ref": "ref.csv",
    }
    (scenario_dir / "scenario.json").write_text(json.dumps(scenario, indent=2), encoding="utf-8")
    return scenario_dir


def main() -> None:
    args = parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)
    raw = fetch_groundtruth(args.url)

    target_start = args.start_time
    target_end = target_start + args.duration_s
    mask = (raw[:, 0] >= target_start) & (raw[:, 0] <= target_end)
    segment = raw[mask]
    if len(segment) < 2:
        raise RuntimeError(f"segment too short in [{target_start}, {target_end}]")

    raw_t = segment[:, 0]
    raw_xyz = segment[:, 1:4]
    target_t = np.arange(0.0, (args.N + 1) * args.dt, args.dt)
    if target_t[-1] > raw_t[-1] - raw_t[0] + 1e-9:
        raise RuntimeError("insufficient segment duration for requested N/dt")

    t_norm = raw_t - raw_t[0]
    x = np.interp(target_t, t_norm, raw_xyz[:, 0])
    y = np.interp(target_t, t_norm, raw_xyz[:, 1])
    z = np.interp(target_t, t_norm, raw_xyz[:, 2])
    xyz = np.stack([x, y, z], axis=1)

    vxyz = np.stack([
        compute_velocity(target_t, xyz[:, 0]),
        compute_velocity(target_t, xyz[:, 1]),
        compute_velocity(target_t, xyz[:, 2]),
    ], axis=1)

    scenario_dir = write_scenario(args, target_t, xyz, vxyz)
    print(f"wrote scenario {scenario_dir}")


if __name__ == "__main__":
    main()
