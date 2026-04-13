#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


if __package__ in (None, ""):
    ROOT = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(ROOT))
    from candidate_specs import load_candidate
else:
    ROOT = Path(__file__).resolve().parents[2]
    from ...candidate_specs import load_candidate


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run one native CasADi C++ benchmark candidate.")
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--build-dir", type=Path, default=None)
    parser.add_argument("--binary", type=Path, default=None)
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def run_command(cmd: list[str]) -> None:
    print("+", " ".join(str(part) for part in cmd))
    subprocess.run(cmd, check=True)


def ensure_binary(binary: Path | None, build_dir: Path, candidate: dict) -> Path:
    if binary is not None:
        return binary if binary.is_absolute() else ROOT / binary

    model_family = candidate["model_family"]
    if model_family == "kinematic_bicycle_track_following":
        target = "casadi_kinematic_bicycle_benchmark"
    elif model_family == "double_integrator_3d_tracking":
        target = "casadi_double_integrator_3d_benchmark"
    else:
        raise SystemExit(f"unsupported model_family for CasADi target: {model_family}")

    cache_exists = (build_dir / "CMakeCache.txt").exists()
    if not cache_exists:
        run_command(["cmake", "-S", str(ROOT), "-B", str(build_dir)])
    try:
        run_command(["cmake", "--build", str(build_dir), "--target", target, "-j"])
    except subprocess.CalledProcessError:
        if cache_exists:
            run_command(["cmake", "-S", str(ROOT), "-B", str(build_dir)])
            run_command(["cmake", "--build", str(build_dir), "--target", target, "-j"])
        else:
            raise
    return build_dir / target


def main() -> int:
    args = parse_args()
    candidate = load_candidate(args.candidate)
    candidate_name = candidate["name"]
    build_dir = args.build_dir or (ROOT / ".build" / "casadi_assets" / candidate_name)
    output = args.output or (ROOT / "results" / "raw" / "casadi" / f"{candidate_name}.csv")
    output.parent.mkdir(parents=True, exist_ok=True)
    binary = ensure_binary(args.binary, build_dir, candidate)
    asset_path = ROOT / candidate["asset"]
    cmd = [str(binary), "--asset", str(asset_path), "--output", str(output), "--dt", str(candidate["dt"]), "--horizon", str(candidate["horizon"]), "--steps", str(args.steps if args.steps is not None else candidate["closed_loop_steps"])]
    if "target_speed_mps" in candidate:
        cmd += ["--target-speed", str(candidate["target_speed_mps"])]
    vehicle = candidate.get("vehicle", {})
    if isinstance(vehicle, dict):
        if "wheelbase_m" in vehicle:
            cmd += ["--wheelbase", str(vehicle["wheelbase_m"])]
        if "delta_max_rad" in vehicle:
            cmd += ["--delta-max", str(vehicle["delta_max_rad"])]
    run_command(cmd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
