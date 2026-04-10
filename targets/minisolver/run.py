#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from cases import backend_case_names


def default_minisolver_source_dir() -> Path:
    local_checkout = ROOT.parent / "MiniSolver"
    if (local_checkout / "CMakeLists.txt").exists():
        return local_checkout
    submodule_dir = ROOT / "third_party" / "MiniSolver"
    if (submodule_dir / "CMakeLists.txt").exists():
        return submodule_dir
    return local_checkout


def default_acados_repo() -> Path:
    submodule_dir = ROOT / "third_party" / "acados"
    if (submodule_dir / "CMakeLists.txt").exists():
        return submodule_dir
    return Path("/tmp/acados-official")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a MiniSolver native benchmark target.")
    parser.add_argument("--case", choices=backend_case_names("minisolver"), required=True)
    parser.add_argument("--minisolver-source-dir", type=Path, default=default_minisolver_source_dir())
    parser.add_argument("--acados-repo", type=Path, default=default_acados_repo())
    parser.add_argument("--build-dir", type=Path, default=None)
    parser.add_argument("--binary", type=Path, default=None)
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--refresh-assets", action="store_true", help="Re-export official acados case assets before building.")
    return parser.parse_args()


def run_command(cmd: list[str]) -> None:
    print("+", " ".join(str(part) for part in cmd))
    subprocess.run(cmd, check=True)


def ensure_official_assets(case_name: str, acados_repo: Path, refresh: bool) -> dict:
    metadata_path = ROOT / "out" / "acados" / case_name / "metadata.json"
    if case_name in {"race_cars", "quadrotor_nav"} and (refresh or not metadata_path.exists()):
        script = ROOT / "backends" / "acados" / "run.py"
        cmd = [
            sys.executable,
            str(script),
            "--case",
            case_name,
            "--acados-repo",
            str(acados_repo.resolve()),
            "--output-root",
            str(ROOT / "out" / "acados"),
            "--prepare-only",
            "--skip-build",
        ]
        run_command(cmd)

    if metadata_path.exists():
        return json.loads(metadata_path.read_text(encoding="utf-8"))
    return {}


def load_race_data(acados_repo: Path) -> dict:
    track_path = acados_repo / "examples" / "acados_python" / "race_cars" / "tracks" / "LMS_Track.txt"
    track = np.loadtxt(track_path)
    return {
        "horizon": 50,
        "steps": 500,
        "tf": 1.0,
        "pathlength": float(track[-1, 0]),
        "sref_n": 3.0,
        "x0": [-2.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }


def load_quad_data() -> dict:
    g0 = 9.80665
    mq = 31e-3
    ct = 3.25e-4
    u_hov = int(np.sqrt(0.25 * 1e6 * mq * g0 / ct)) / 1000
    return {
        "horizon": 50,
        "steps": 2250,
        "tf": 1.0,
        "u_hov": float(u_hov),
        "s_ref": 0.1875,
        "s_max": 5.9,
        "init_zeta": [
            0.05, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.02, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            float(u_hov), float(u_hov), float(u_hov), float(u_hov),
        ],
    }


def load_chain_data() -> dict:
    return {
        "steps": 25,
        "n_mass": 5,
        "wall_count": 4,
        "ts": 0.2,
        "tf": 8.0,
        "y_pos_wall": -0.05,
        "perturb_scale": 0.01,
        "seed": 50,
        "u_init": [-1.0, 1.0, 1.0],
        "xrest": [
            0.1914638616713326, 0.0, -0.5145549913187678,
            0.396, 0.0, -0.6976151145457183,
            0.6005361383286675, 0.0, -0.5145549913187678,
            0.792, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ],
    }


def c_array(values: list[float] | tuple[float, ...]) -> str:
    return ", ".join(f"{float(v):.17g}" for v in values)


def load_official_case_metadata(acados_repo: Path, refresh_case: str | None) -> dict[str, dict]:
    metadata = {
        "race_cars": load_race_data(acados_repo),
        "quadrotor_nav": load_quad_data(),
        "chain_mass": load_chain_data(),
    }
    for case_name in ("race_cars", "quadrotor_nav"):
        exported = ensure_official_assets(case_name, acados_repo, refresh=refresh_case == case_name)
        if case_name == "race_cars":
            metadata[case_name].update({
                "horizon": int(exported.get("horizon", metadata[case_name]["horizon"])),
                "steps": int(exported.get("defaults", {}).get("steps", metadata[case_name]["steps"])),
                "tf": float(exported.get("tf", metadata[case_name]["tf"])),
                "pathlength": float(exported.get("track_pathlength", metadata[case_name]["pathlength"])),
                "sref_n": float(exported.get("sref_n", metadata[case_name]["sref_n"])),
                "x0": exported.get("x0_init", metadata[case_name]["x0"]),
            })
        else:
            metadata[case_name].update({
                "horizon": int(exported.get("horizon", metadata[case_name]["horizon"])),
                "steps": int(exported.get("defaults", {}).get("steps", metadata[case_name]["steps"])),
                "tf": float(exported.get("tf", metadata[case_name]["tf"])),
                "u_hov": float(exported.get("u_hov", metadata[case_name]["u_hov"])),
                "s_ref": float(exported.get("s_ref", metadata[case_name]["s_ref"])),
                "s_max": float(exported.get("s_max", metadata[case_name]["s_max"])),
                "init_zeta": exported.get("init_zeta", metadata[case_name]["init_zeta"]),
            })
    return metadata


def generate_native_model(case_name: str, minisolver_source_dir: Path, acados_repo: Path) -> None:
    script_dir = ROOT / "targets" / "minisolver" / "models"
    script_map = {
        "pendulum_on_cart": script_dir / "acados_pendulum" / "generate_model.py",
    }
    if case_name not in script_map:
        return
    script_path = script_map[case_name]
    env = os.environ.copy()
    env["MINISOLVER_SOURCE_DIR"] = str(minisolver_source_dir.resolve())
    env["ACADOS_REPO"] = str(acados_repo.resolve())
    cmd = [sys.executable, str(script_path)]
    print("+", " ".join(str(part) for part in cmd))
    subprocess.run(cmd, cwd=ROOT, env=env, check=True)


def write_official_case_header(metadata_by_case: dict[str, dict]) -> None:
    header_path = ROOT / "targets" / "minisolver" / "generated" / "official_case_data.h"
    race_metadata = metadata_by_case["race_cars"]
    quad_metadata = metadata_by_case["quadrotor_nav"]
    chain_metadata = metadata_by_case["chain_mass"]

    pendulum = {
        "horizon": 40,
        "steps": 100,
        "tf": 0.8,
        "x0": [0.0, 3.14159265358979323846, 0.0, 0.0],
    }
    race = {
        "horizon": int(race_metadata["horizon"]),
        "steps": int(race_metadata["steps"]),
        "tf": float(race_metadata["tf"]),
        "pathlength": float(race_metadata["pathlength"]),
        "sref_n": float(race_metadata["sref_n"]),
        "x0": race_metadata["x0"],
    }
    quad = {
        "horizon": int(quad_metadata["horizon"]),
        "steps": int(quad_metadata["steps"]),
        "tf": float(quad_metadata["tf"]),
        "u_hov": float(quad_metadata["u_hov"]),
        "s_ref": float(quad_metadata["s_ref"]),
        "s_max": float(quad_metadata["s_max"]),
        "init_zeta": quad_metadata["init_zeta"],
    }
    chain = {
        "steps": int(chain_metadata["steps"]),
        "n_mass": int(chain_metadata["n_mass"]),
        "wall_count": int(chain_metadata["wall_count"]),
        "ts": float(chain_metadata["ts"]),
        "tf": float(chain_metadata["tf"]),
        "y_pos_wall": float(chain_metadata["y_pos_wall"]),
        "perturb_scale": float(chain_metadata["perturb_scale"]),
        "seed": int(chain_metadata["seed"]),
        "u_init": chain_metadata["u_init"],
        "xrest": chain_metadata["xrest"],
    }

    text = f"""#pragma once

#include <array>

namespace minisolver_bench::generated {{

inline constexpr int pendulum_horizon = {pendulum["horizon"]};
inline constexpr int pendulum_steps = {pendulum["steps"]};
inline constexpr double pendulum_tf = {pendulum["tf"]:.17g};
inline constexpr std::array<double, 4> pendulum_x0 = {{{c_array(pendulum["x0"])}}};

inline constexpr int race_cars_horizon = {race["horizon"]};
inline constexpr int race_cars_steps = {race["steps"]};
inline constexpr double race_cars_tf = {race["tf"]:.17g};
inline constexpr double race_cars_pathlength = {race["pathlength"]:.17g};
inline constexpr double race_cars_sref_n = {race["sref_n"]:.17g};
inline constexpr std::array<double, 6> race_cars_x0 = {{{c_array(race["x0"])}}};

inline constexpr int quadrotor_nav_horizon = {quad["horizon"]};
inline constexpr int quadrotor_nav_steps = {quad["steps"]};
inline constexpr double quadrotor_nav_tf = {quad["tf"]:.17g};
inline constexpr double quadrotor_nav_u_hov = {quad["u_hov"]:.17g};
inline constexpr double quadrotor_nav_s_ref = {quad["s_ref"]:.17g};
inline constexpr double quadrotor_nav_s_max = {quad["s_max"]:.17g};
inline constexpr std::array<double, 20> quadrotor_nav_init_zeta = {{{c_array(quad["init_zeta"])}}};

inline constexpr int chain_mass_steps = {chain["steps"]};
inline constexpr int chain_mass_n_mass = {chain["n_mass"]};
inline constexpr int chain_mass_wall_count = {chain["wall_count"]};
inline constexpr double chain_mass_ts = {chain["ts"]:.17g};
inline constexpr double chain_mass_tf = {chain["tf"]:.17g};
inline constexpr double chain_mass_y_pos_wall = {chain["y_pos_wall"]:.17g};
inline constexpr double chain_mass_perturb_scale = {chain["perturb_scale"]:.17g};
inline constexpr int chain_mass_seed = {chain["seed"]};
inline constexpr std::array<double, 3> chain_mass_u_init = {{{c_array(chain["u_init"])}}};
inline constexpr std::array<double, 21> chain_mass_xrest = {{{c_array(chain["xrest"])}}};

}}  // namespace minisolver_bench::generated
"""
    header_path.write_text(text, encoding="utf-8")


def ensure_binary(
    case_name: str,
    binary: Path | None,
    build_dir: Path,
    minisolver_source_dir: Path,
) -> Path:
    if binary is not None:
        return binary if binary.is_absolute() else ROOT / binary

    build_dir.mkdir(parents=True, exist_ok=True)
    minisolver_source_dir = minisolver_source_dir.resolve()
    if not (minisolver_source_dir / "CMakeLists.txt").exists():
        raise SystemExit(f"MiniSolver source checkout not found: {minisolver_source_dir}")

    configure_cmd = [
        "cmake",
        "-S",
        str(ROOT),
        "-B",
        str(build_dir),
        f"-DMINISOLVER_SOURCE_DIR={minisolver_source_dir}",
        f"-DMINISOLVER_CASE={case_name}",
        f"-DACADOS_OUTPUT_ROOT={ROOT / 'out' / 'acados'}",
    ]
    build_cmd = [
        "cmake",
        "--build",
        str(build_dir),
        "--target",
        "minisolver_official_case_benchmark",
        "-j",
    ]
    run_command(configure_cmd)
    run_command(build_cmd)
    return build_dir / "minisolver_official_case_benchmark"


def failure_record_path(output: Path) -> Path:
    return output.with_suffix(".failure.json")


def write_failure_record(output: Path, case_name: str, stage: str, error: str) -> None:
    record = {
        "backend": "minisolver",
        "backend_family": "minisolver",
        "case_name": case_name,
        "file": str(output.relative_to(ROOT)),
        "steps": 0,
        "success_rate": 0.0,
        "failure_stage": stage,
        "status_note": error,
    }
    path = failure_record_path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    metadata_by_case = load_official_case_metadata(args.acados_repo, args.case if args.refresh_assets else None)
    write_official_case_header(metadata_by_case)

    build_dir = args.build_dir or (ROOT / ".build" / "minisolver" / args.case)
    output = args.output or (ROOT / "results" / "raw" / "minisolver" / f"{args.case}.csv")
    output.parent.mkdir(parents=True, exist_ok=True)
    failure_path = failure_record_path(output)

    try:
        generate_native_model(args.case, args.minisolver_source_dir, args.acados_repo)
        binary = ensure_binary(
            args.case,
            args.binary,
            build_dir,
            args.minisolver_source_dir,
        )
    except subprocess.CalledProcessError as exc:
        write_failure_record(output, args.case, "build", str(exc))
        return int(exc.returncode or 1)

    if not binary.exists():
        write_failure_record(output, args.case, "build", f"binary not found: {binary}")
        raise SystemExit(f"MiniSolver benchmark binary not found: {binary}")

    if failure_path.exists():
        failure_path.unlink()

    cmd = [str(binary), "--output", str(output)]
    if args.steps is not None:
        cmd += ["--steps", str(args.steps)]

    run_command(cmd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
