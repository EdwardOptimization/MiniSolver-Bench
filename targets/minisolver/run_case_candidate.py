#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def default_minisolver_source_dir() -> Path:
    local_checkout = ROOT.parent / "MiniSolver"
    if (local_checkout / "CMakeLists.txt").exists():
        return local_checkout
    submodule_dir = ROOT / "third_party" / "MiniSolver"
    if (submodule_dir / "CMakeLists.txt").exists():
        return submodule_dir
    return local_checkout


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a MiniSolver benchmark candidate derived from public reference assets.")
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--minisolver-source-dir", type=Path, default=default_minisolver_source_dir())
    parser.add_argument("--build-dir", type=Path, default=None)
    parser.add_argument("--binary", type=Path, default=None)
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def run_command(cmd: list[str]) -> None:
    print("+", " ".join(str(part) for part in cmd))
    subprocess.run(cmd, check=True)


from candidate_specs import load_candidate


def ensure_binary(binary: Path | None, build_dir: Path, minisolver_source_dir: Path, candidate: dict) -> Path:
    if binary is not None:
        return binary if binary.is_absolute() else ROOT / binary

    model_family = candidate["model_family"]
    if model_family == "kinematic_bicycle_track_following":
        target = "minisolver_kinematic_bicycle_benchmark"
    elif model_family == "double_integrator_3d_tracking":
        target = "minisolver_double_integrator_3d_benchmark"
    else:
        raise SystemExit(f"unsupported model_family for MiniSolver target: {model_family}")

    configure_cmd = [
        "cmake",
        "-S",
        str(ROOT),
        "-B",
        str(build_dir),
        f"-DMINISOLVER_SOURCE_DIR={minisolver_source_dir.resolve()}",
        "-DALTRO_SOURCE_DIR=",
        "-DCASADI_ROOT=",
        "-DMINISOLVER_BUILD_TESTS=OFF",
        "-DMINISOLVER_BUILD_EXAMPLES=OFF",
        "-DMINISOLVER_BUILD_TOOLS=OFF",
        "-DMINISOLVER_FETCH_DEPS=OFF",
        "-DCMAKE_BUILD_TYPE=Release",
    ]
    build_cmd = [
        "cmake",
        "--build",
        str(build_dir),
        "--target",
        target,
        "-j",
    ]
    cache_exists = (build_dir / "CMakeCache.txt").exists()
    if not cache_exists:
        run_command(configure_cmd)
    try:
        run_command(build_cmd)
    except subprocess.CalledProcessError:
        if cache_exists:
            run_command(configure_cmd)
            run_command(build_cmd)
        else:
            raise
    return build_dir / target


def main() -> int:
    args = parse_args()
    candidate = load_candidate(args.candidate)
    candidate_name = candidate["name"]
    build_dir = args.build_dir or (ROOT / ".build" / "minisolver_assets" / candidate_name)
    output = args.output or (ROOT / "results" / "raw" / "minisolver" / f"{candidate_name}.csv")
    output.parent.mkdir(parents=True, exist_ok=True)

    binary = ensure_binary(args.binary, build_dir, args.minisolver_source_dir, candidate)
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
