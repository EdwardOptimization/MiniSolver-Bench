#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


if __package__ in (None, ""):
    ROOT = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(ROOT))
    from candidate_specs import load_candidate, list_candidate_names
else:
    ROOT = Path(__file__).resolve().parents[2]
    from ...candidate_specs import load_candidate, list_candidate_names


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run all supported ALTRO asset-derived benchmark candidates.")
    parser.add_argument("--altro-source-dir", type=Path, default=None)
    parser.add_argument("--build-dir", type=Path, default=None)
    parser.add_argument("--steps", type=int, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    script = Path(__file__).resolve().parent / "run_candidate.py"
    for name in list_candidate_names():
        candidate = load_candidate(name)
        if candidate["model_family"] != "double_integrator_3d_tracking":
            continue

        cmd = [sys.executable, str(script), "--candidate", name]
        if args.altro_source_dir is not None:
            cmd += ["--altro-source-dir", str(args.altro_source_dir)]
        if args.build_dir is not None:
            cmd += ["--build-dir", str(args.build_dir)]
        if args.steps is not None:
            cmd += ["--steps", str(args.steps)]
        print("+", " ".join(str(part) for part in cmd))
        subprocess.run(cmd, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
