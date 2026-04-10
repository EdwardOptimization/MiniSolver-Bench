#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


if __package__ in (None, ""):
    ROOT = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(ROOT))
    from candidate_specs import list_candidate_names
else:
    ROOT = Path(__file__).resolve().parents[2]
    from ...candidate_specs import list_candidate_names


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run all acados asset-derived benchmark candidates.")
    parser.add_argument("--acados-repo", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    script = Path(__file__).resolve().parent / "run_candidate.py"
    for name in list_candidate_names():
        cmd = [sys.executable, str(script), "--candidate", name]
        if args.acados_repo is not None:
            cmd += ["--acados-repo", str(args.acados_repo)]
        print("+", " ".join(str(part) for part in cmd))
        subprocess.run(cmd, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
