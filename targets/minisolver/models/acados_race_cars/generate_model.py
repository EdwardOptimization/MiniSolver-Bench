#!/usr/bin/env python3
"""Materialize the official race_cars generated header into callback-window form."""
from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from targets.minisolver.models.callback_header_patch import ensure_race_callback_header  # noqa: E402


def main() -> int:
    header_path = Path(__file__).resolve().parents[2] / "generated" / "racecarsmodel.h"
    ensure_race_callback_header(header_path)
    print(f"Generated callback-window RaceCarsModel in {header_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
