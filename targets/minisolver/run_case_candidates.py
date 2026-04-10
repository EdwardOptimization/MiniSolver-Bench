#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run all MiniSolver benchmark candidates.")
    parser.add_argument("--minisolver-source-dir", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    script = Path(__file__).resolve().parent / "run_case_candidate.py"
    for candidate in sorted((ROOT / "case_candidates").glob("*.json")):
        cmd = [sys.executable, str(script), "--candidate", str(candidate)]
        if args.minisolver_source_dir is not None:
            cmd += ["--minisolver-source-dir", str(args.minisolver_source_dir)]
        print("+", " ".join(cmd))
        subprocess.run(cmd, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
