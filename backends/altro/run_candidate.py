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


def default_altro_source_dir() -> Path:
    vendored_checkout = ROOT / "third_party" / "ALTRO"
    if (vendored_checkout / "CMakeLists.txt").exists():
        return vendored_checkout
    local_checkout = ROOT.parent / "ALTRO"
    if (local_checkout / "CMakeLists.txt").exists():
        return local_checkout
    return vendored_checkout


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run one native ALTRO C++ benchmark candidate.")
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--altro-source-dir", type=Path, default=default_altro_source_dir())
    parser.add_argument("--build-dir", type=Path, default=None)
    parser.add_argument("--binary", type=Path, default=None)
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def run_command(cmd: list[str]) -> None:
    print("+", " ".join(str(part) for part in cmd))
    subprocess.run(cmd, check=True)


def ensure_binary(binary: Path | None, build_dir: Path, altro_source_dir: Path, candidate: dict) -> Path:
    if binary is not None:
        return binary if binary.is_absolute() else ROOT / binary

    model_family = candidate["model_family"]
    if model_family != "double_integrator_3d_tracking":
        raise SystemExit(f"ALTRO backend only supports double_integrator_3d_tracking, got {model_family}")
    target = "altro_double_integrator_3d_benchmark"

    if not (altro_source_dir / "CMakeLists.txt").exists():
        raise SystemExit(
            "ALTRO_SOURCE_DIR does not point to an ALTRO checkout: "
            f"{altro_source_dir}\n"
            "Initialize the default checkout with: "
            "git submodule update --init --recursive third_party/ALTRO"
        )

    configure_cmd = [
        "cmake",
        "-S",
        str(ROOT),
        "-B",
        str(build_dir),
        "-DCMAKE_BUILD_TYPE=Release",
        f"-DALTRO_SOURCE_DIR={altro_source_dir.resolve()}",
        "-DMINISOLVER_SOURCE_DIR=",
        "-DCASADI_ROOT=",
    ]
    build_cmd = ["cmake", "--build", str(build_dir), "--target", target, "-j"]
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
    build_dir = args.build_dir or (ROOT / ".build" / "altro_assets")
    output = args.output or (ROOT / "results" / "raw" / "altro" / f"{candidate_name}.csv")
    output.parent.mkdir(parents=True, exist_ok=True)

    binary = ensure_binary(args.binary, build_dir, args.altro_source_dir, candidate)
    asset_path = ROOT / candidate["asset"]
    cmd = [
        str(binary),
        "--asset",
        str(asset_path),
        "--output",
        str(output),
        "--dt",
        str(candidate["dt"]),
        "--horizon",
        str(candidate["horizon"]),
        "--steps",
        str(args.steps if args.steps is not None else candidate["closed_loop_steps"]),
    ]
    run_command(cmd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
