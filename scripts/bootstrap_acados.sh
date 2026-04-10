#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ACADOS_DIR="${ROOT_DIR}/third_party/acados"

if [[ ! -f "${ACADOS_DIR}/CMakeLists.txt" ]]; then
  echo "acados submodule not found: ${ACADOS_DIR}" >&2
  exit 1
fi

git -C "${ROOT_DIR}" submodule update --init --recursive third_party/acados

export ACADOS_SOURCE_DIR="${ACADOS_DIR}"

python3 - <<'PY'
import os
import sys
from pathlib import Path

acados_dir = Path(os.environ["ACADOS_SOURCE_DIR"])
sys.path.insert(0, str(acados_dir / "interfaces" / "acados_template"))
from acados_template.utils import get_tera

print(get_tera(force_download=True))
PY

cmake -S "${ACADOS_DIR}" -B "${ACADOS_DIR}/build" -DBUILD_SHARED_LIBS=ON -DACADOS_WITH_OPENMP=OFF
cmake --build "${ACADOS_DIR}/build" -j"$(nproc)"
cmake --install "${ACADOS_DIR}/build"
