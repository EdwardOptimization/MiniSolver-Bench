#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from cases import load_case, list_case_names


ROOT = Path(__file__).resolve().parent


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description="Unified benchmark entry point.")
    parser.add_argument("--backend", choices=["minisolver", "acados", "casadi"], required=True)
    parser.add_argument("--case", choices=list_case_names(), required=True)
    return parser.parse_known_args()


def main() -> int:
    args, rest = parse_args()
    case = load_case(args.case)
    backend = case.get("backends", {}).get(args.backend, {})
    if not backend.get("supported"):
        raise SystemExit(f"case '{args.case}' is not supported by backend '{args.backend}'")

    if args.backend == "minisolver":
        script = ROOT / "targets" / "minisolver" / "run.py"
    else:
        script = ROOT / "backends" / args.backend / "run.py"
    cmd = [sys.executable, str(script), "--case", args.case, *rest]
    print("+", " ".join(cmd))
    subprocess.run(cmd, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
