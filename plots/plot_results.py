#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot benchmark latency from a CSV file.")
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--y", default="time_ms")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    df = pd.read_csv(args.input)
    if args.y not in df.columns:
        raise SystemExit(f"column '{args.y}' not found in {args.input}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(8, 4))
    plt.plot(df.index, df[args.y])
    plt.xlabel("step")
    plt.ylabel(args.y)
    plt.tight_layout()
    plt.savefig(args.output, dpi=150)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
