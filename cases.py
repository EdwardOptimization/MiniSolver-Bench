from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
CASE_DIR = ROOT / "cases"


def list_case_names() -> list[str]:
    return sorted(path.stem for path in CASE_DIR.glob("*.json"))


def load_case(name: str) -> dict:
    path = CASE_DIR / f"{name}.json"
    if not path.exists():
        raise FileNotFoundError(f"benchmark case not found: {path}")
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def backend_case_names(backend_name: str) -> list[str]:
    names: list[str] = []
    for name in list_case_names():
        case = load_case(name)
        backend = case.get("backends", {}).get(backend_name, {})
        if backend.get("supported"):
            names.append(name)
    return names
