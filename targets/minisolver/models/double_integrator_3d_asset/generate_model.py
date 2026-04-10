from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from targets.minisolver.models.common import ensure_minisolver_python_path


ensure_minisolver_python_path()

from minisolver.MiniModel import OptimalControlModel  # noqa: E402


ACC_MAX = 15.0


if __name__ == "__main__":
    model = OptimalControlModel("DoubleIntegrator3DTrackingModel")

    x, y, z, vx, vy, vz = model.state("x", "y", "z", "vx", "vy", "vz")
    ax, ay, az = model.control("ax", "ay", "az")

    x_ref = model.parameter("x_ref")
    y_ref = model.parameter("y_ref")
    z_ref = model.parameter("z_ref")
    vx_ref = model.parameter("vx_ref")
    vy_ref = model.parameter("vy_ref")
    vz_ref = model.parameter("vz_ref")

    model.set_dynamics(x, vx)
    model.set_dynamics(y, vy)
    model.set_dynamics(z, vz)
    model.set_dynamics(vx, ax)
    model.set_dynamics(vy, ay)
    model.set_dynamics(vz, az)

    model.minimize(15.0 * (x - x_ref) ** 2)
    model.minimize(15.0 * (y - y_ref) ** 2)
    model.minimize(15.0 * (z - z_ref) ** 2)
    model.minimize(2.5 * (vx - vx_ref) ** 2)
    model.minimize(2.5 * (vy - vy_ref) ** 2)
    model.minimize(2.5 * (vz - vz_ref) ** 2)
    model.minimize(0.05 * ax**2)
    model.minimize(0.05 * ay**2)
    model.minimize(0.05 * az**2)

    model.subject_to(ax - ACC_MAX <= 0)
    model.subject_to(-ACC_MAX - ax <= 0)
    model.subject_to(ay - ACC_MAX <= 0)
    model.subject_to(-ACC_MAX - ay <= 0)
    model.subject_to(az - ACC_MAX <= 0)
    model.subject_to(-ACC_MAX - az <= 0)

    output_dir = ROOT / "targets" / "minisolver" / "generated"
    model.generate(str(output_dir))
    print(f"Generated DoubleIntegrator3DTrackingModel in {output_dir}")
