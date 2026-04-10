#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from cases import backend_case_names


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run all official MiniSolver benchmark cases.")
    parser.add_argument("--minisolver-source-dir", type=Path, default=None)
    parser.add_argument("--acados-repo", type=Path, default=None)
    parser.add_argument("--refresh-assets", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    script = Path(__file__).resolve().parent / "run.py"
    for case_name in backend_case_names("minisolver"):
        cmd = [
            sys.executable,
            str(script),
            "--case",
            case_name,
        ]
        if args.minisolver_source_dir is not None:
            cmd += ["--minisolver-source-dir", str(args.minisolver_source_dir)]
        if args.acados_repo is not None:
            cmd += ["--acados-repo", str(args.acados_repo)]
        if args.refresh_assets:
            cmd.append("--refresh-assets")
        print("+", " ".join(cmd))
        subprocess.run(cmd, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
