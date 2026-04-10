#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


if __package__ in (None, ""):
    ROOT = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(ROOT))
    from cases import backend_case_names
else:
    ROOT = Path(__file__).resolve().parents[2]
    from ...cases import backend_case_names


def default_acados_repo() -> Path:
    submodule_dir = ROOT / "third_party" / "acados"
    if (submodule_dir / "CMakeLists.txt").exists():
        return submodule_dir
    return Path("/tmp/acados-official")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run all official CasADi SQP benchmark cases.")
    parser.add_argument("--acados-repo", type=Path, default=default_acados_repo())
    parser.add_argument("--steps", type=int, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    script = Path(__file__).resolve().parent / "run.py"
    for case_name in backend_case_names("casadi"):
        cmd = [sys.executable, str(script), "--case", case_name, "--acados-repo", str(args.acados_repo)]
        if args.steps is not None:
            cmd += ["--steps", str(args.steps)]
        print("+", " ".join(cmd))
        subprocess.run(cmd, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
