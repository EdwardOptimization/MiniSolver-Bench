#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from io import StringIO
from pathlib import Path
from typing import Tuple

import numpy as np
import pandas as pd
import requests


ROOT = Path(__file__).resolve().parents[1]
NGSIM_API = "https://data.transportation.gov/resource/8ect-6jqj.csv"
FT_TO_M = 0.3048


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Import a small NGSIM segment into scenario format.")
    parser.add_argument("--vehicle-id", type=int, required=True)
    parser.add_argument("--location", default="peachtree")
    parser.add_argument("--start-time-ms", type=int, required=True)
    parser.add_argument("--name", required=True)
    parser.add_argument("--output-root", type=Path, default=ROOT / "scenarios")
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--N", type=int, default=50)
    return parser.parse_args()


def fetch_rows(vehicle_id: int, location: str, start_ms: int, end_ms: int) -> pd.DataFrame:
    where = (
        f"vehicle_id = {vehicle_id} and location = '{location}' and "
        f"global_time >= {start_ms} and global_time <= {end_ms}"
    )
    params = {
        "$select": "global_time,local_x,local_y,v_vel",
        "$where": where,
        "$order": "global_time ASC",
        "$limit": 100000,
    }
    resp = requests.get(NGSIM_API, params=params, timeout=60)
    resp.raise_for_status()
    df = pd.read_csv(StringIO(resp.text))
    return df


def write_scenario(scenario_name: str, rows: pd.DataFrame, args: argparse.Namespace) -> Tuple[Path, float]:
    rows = rows.sort_values("global_time").reset_index(drop=True)
    t0 = rows["global_time"].to_numpy(dtype=float)
    x0 = rows["local_x"].to_numpy(dtype=float) * FT_TO_M
    y0 = rows["local_y"].to_numpy(dtype=float) * FT_TO_M
    v0 = rows["v_vel"].to_numpy(dtype=float) * FT_TO_M
    t_sec = (t0 - t0[0]) / 1000.0

    target_t = np.arange(0.0, args.N * args.dt + 1e-12, args.dt)
    if target_t[-1] > t_sec[-1] + 1e-9:
        raise RuntimeError("insufficient sample duration for requested N/dt")

    x_ref = np.interp(target_t, t_sec, x0)
    y_ref = np.interp(target_t, t_sec, y0)
    v_ref = np.interp(target_t, t_sec, v0)

    yaw_ref = np.zeros_like(target_t)
    dx = np.diff(x_ref)
    dy = np.diff(y_ref)
    yaw = np.arctan2(dy, dx)
    yaw_ref[:-1] = yaw
    yaw_ref[-1] = yaw_ref[-2] if len(yaw_ref) > 1 else 0.0

    scenario_dir = args.output_root / scenario_name
    scenario_dir.mkdir(parents=True, exist_ok=True)
    ref_csv = scenario_dir / "ref.csv"

    import_csv = {
        "t": [f"{t:.17g}" for t in target_t],
        "x_ref": [f"{v:.17g}" for v in x_ref],
        "y_ref": [f"{v:.17g}" for v in y_ref],
        "yaw_ref": [f"{v:.17g}" for v in yaw_ref],
        "v_ref": [f"{v:.17g}" for v in v_ref],
        "w_left": [f"{1.85:.17g}"] * len(target_t),
        "w_right": [f"{1.85:.17g}"] * len(target_t),
    }
    pd.DataFrame(import_csv).to_csv(ref_csv, index=False)

    scenario = {
        "name": scenario_name,
        "domain": "driving",
        "model": "kinematic_bicycle_v1",
        "dt": args.dt,
        "N": args.N,
        "x0": [
            float(x_ref[0]),
            float(y_ref[0]),
            float(yaw_ref[0]),
            float(v_ref[0]),
            0.0,
        ],
        "ref": "ref.csv",
    }
    (scenario_dir / "scenario.json").write_text(json.dumps(scenario, indent=2), encoding="utf-8")
    return scenario_dir, float(v_ref[0])


def main() -> None:
    args = parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)
    start = args.start_time_ms
    end = int(args.start_time_ms + args.duration_s * 1000)
    rows = fetch_rows(args.vehicle_id, args.location, start, end)
    if rows.empty:
        raise RuntimeError(f"no rows returned for vehicle {args.vehicle_id} at {start}-{end}")
    if len(rows) < 2:
        raise RuntimeError("not enough rows after query for resampling")
    out_dir, _ = write_scenario(args.name, rows, args)
    print(f"wrote scenario {out_dir}")


if __name__ == "__main__":
    main()
