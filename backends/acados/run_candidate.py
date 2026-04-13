#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ctypes
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np


if __package__ in (None, ""):
    ROOT = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(ROOT))
    from candidate_specs import load_candidate
else:
    ROOT = Path(__file__).resolve().parents[2]
    from ...candidate_specs import load_candidate


def fail(message: str) -> "NoReturn":
    raise SystemExit(message)


def import_python_dependencies():
    try:
        import numpy as np  # noqa: F401
        import scipy.linalg  # noqa: F401
        import casadi as ca  # noqa: F401
    except ModuleNotFoundError as exc:
        fail(
            "Missing Python dependency for acados export: "
            f"{exc.name}. Install the official acados Python dependencies first."
        )


def configure_acados_python(acados_repo: Path) -> None:
    acados_repo = acados_repo.resolve()
    template_root = acados_repo / "interfaces" / "acados_template"
    if not template_root.exists():
        fail(f"acados_template path not found: {template_root}")
    sys.path.insert(0, str(template_root))
    lib_path = acados_repo / "lib"
    include_path = acados_repo / "include"
    os.environ["ACADOS_SOURCE_DIR"] = str(acados_repo)
    os.environ["ACADOS_LIB_PATH"] = str(lib_path)
    os.environ["ACADOS_INCLUDE_PATH"] = str(include_path)

    ld_parts = [str(lib_path)]
    if os.environ.get("LD_LIBRARY_PATH"):
        ld_parts.append(os.environ["LD_LIBRARY_PATH"])
    os.environ["LD_LIBRARY_PATH"] = ":".join(ld_parts)

    for shared_lib in ("libblasfeo.so", "libhpipm.so", "libacados.so"):
        lib_file = lib_path / shared_lib
        if not lib_file.exists():
            fail(f"required acados shared library not found: {lib_file}")
        ctypes.CDLL(str(lib_file), mode=ctypes.RTLD_GLOBAL)


def dump_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2, sort_keys=True)
        fh.write("\n")


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def run_command(cmd: list[str], env: dict[str, str] | None = None) -> None:
    if env is None:
        env = os.environ.copy()
    print("+", " ".join(cmd))
    subprocess.run(cmd, check=True, env=env)


def default_acados_repo() -> Path:
    submodule_dir = ROOT / "third_party" / "acados"
    if (submodule_dir / "CMakeLists.txt").exists():
        return submodule_dir
    return Path("/tmp/acados-official")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export, build and run one asset-derived acados benchmark candidate.")
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--acados-repo", type=Path, default=default_acados_repo())
    parser.add_argument("--output-root", type=Path, default=Path("out/acados_assets"))
    parser.add_argument("--results-root", type=Path, default=Path("results/raw/acados"))
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--skip-export", action="store_true")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--prepare-only", action="store_true")
    return parser.parse_args()


def _model_upper(name: str) -> str:
    return name.upper()


def export_kinematic_bicycle(candidate: dict, case_root: Path) -> dict:
    import casadi as ca
    from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

    dt = float(candidate["dt"])
    horizon = int(candidate["horizon"])
    vehicle = candidate.get("vehicle", {})
    if not isinstance(vehicle, dict):
        vehicle = {}

    x = ca.SX.sym("x")
    y = ca.SX.sym("y")
    psi = ca.SX.sym("psi")
    v = ca.SX.sym("v")
    delta = ca.SX.sym("delta")
    state = ca.vertcat(x, y, psi, v, delta)

    accel = ca.SX.sym("a")
    delta_rate = ca.SX.sym("delta_rate")
    control = ca.vertcat(accel, delta_rate)

    p = ca.SX.sym("p", 8)
    x_ref, y_ref, psi_ref, v_ref, n_x, n_y, w_left, w_right = [p[i] for i in range(8)]

    v_min, v_max = 0.1, 12.0
    wheelbase = float(vehicle.get("wheelbase_m", 0.33))
    delta_max = float(vehicle.get("delta_max_rad", 0.50))
    delta_min = -delta_max

    f_expl = ca.vertcat(
        v * ca.cos(psi),
        v * ca.sin(psi),
        v * ca.tan(delta) / wheelbase,
        accel,
        delta_rate,
    )

    psi_error = ca.atan2(ca.sin(psi - psi_ref), ca.cos(psi - psi_ref))
    lateral = n_x * (x - x_ref) + n_y * (y - y_ref)

    model = AcadosModel()
    model.name = "kinematic_bicycle_asset"
    model.x = state
    model.xdot = ca.SX.sym("xdot", 5)
    model.u = control
    model.p = p
    model.f_expl_expr = f_expl
    model.f_impl_expr = model.xdot - f_expl
    model.cost_y_expr = ca.vertcat(x - x_ref, y - y_ref, psi_error, v - v_ref, delta, accel, delta_rate)
    model.cost_y_expr_e = ca.vertcat(x - x_ref, y - y_ref, psi_error, v - v_ref, delta)
    model.con_h_expr = ca.vertcat(
        lateral - w_left,
        -lateral - w_right,
    )
    model.con_h_expr_e = model.con_h_expr

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = horizon
    ocp.parameter_values = np.zeros((8,))
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.W = np.diag([20.0, 20.0, 5.0, 2.0, 0.5, 0.05, 0.05])
    ocp.cost.W_e = np.diag([20.0, 20.0, 5.0, 2.0, 0.5])
    ocp.cost.yref = np.zeros((7,))
    ocp.cost.yref_e = np.zeros((5,))
    ocp.constraints.x0 = np.zeros((5,))
    ocp.constraints.lbx = np.array([v_min, delta_min])
    ocp.constraints.ubx = np.array([v_max, delta_max])
    ocp.constraints.idxbx = np.array([3, 4], dtype=np.int64)
    ocp.constraints.lbx_e = ocp.constraints.lbx.copy()
    ocp.constraints.ubx_e = ocp.constraints.ubx.copy()
    ocp.constraints.idxbx_e = ocp.constraints.idxbx.copy()

    # Bounds on acceleration and steering rate to keep the closed-loop simulation well-posed and comparable
    # across backends.
    ocp.constraints.lbu = np.array([-15.0, -6.0])
    ocp.constraints.ubu = np.array([15.0, 6.0])
    ocp.constraints.idxbu = np.array([0, 1], dtype=np.int64)

    ocp.constraints.lh = -1e6 * np.ones((2,))
    ocp.constraints.uh = np.zeros((2,))
    ocp.constraints.lh_e = -1e6 * np.ones((2,))
    ocp.constraints.uh_e = np.zeros((2,))

    # Soft slacks for the two lateral track corridor constraints.
    # This mirrors common acados MPC practice (RTI + soft constraints) and avoids QP infeasibility
    # from poor linearizations when starting far from the corridor.
    nsh = 2
    ocp.constraints.idxsh = np.array(range(nsh))
    ocp.constraints.idxsh_e = np.array(range(nsh))
    ocp.constraints.lsh = np.zeros(nsh)
    ocp.constraints.ush = np.zeros(nsh)
    ocp.constraints.lsh_e = np.zeros(nsh)
    ocp.constraints.ush_e = np.zeros(nsh)
    soft_w = 200.0
    ocp.cost.zl = soft_w * np.ones((nsh,))
    ocp.cost.zu = soft_w * np.ones((nsh,))
    ocp.cost.Zl = np.zeros((nsh,))
    ocp.cost.Zu = np.zeros((nsh,))
    ocp.cost.zl_e = ocp.cost.zl.copy()
    ocp.cost.zu_e = ocp.cost.zu.copy()
    ocp.cost.Zl_e = ocp.cost.Zl.copy()
    ocp.cost.Zu_e = ocp.cost.Zu.copy()
    ocp.solver_options.tf = horizon * dt
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_cond_N = horizon
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    # Fastest closed-loop mode in acados is RTI (one SQP iteration split into preparation + feedback).
    # This matches the intended deployment pattern for embedded NMPC.
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.tol = 1e-4
    ocp.solver_options.qp_tol = 1e-4

    generated_dir = case_root / "generated"
    json_file = case_root / "acados_ocp_kinematic_bicycle_asset.json"
    ocp.code_export_directory = str(generated_dir)
    AcadosOcpSolver(ocp, json_file=str(json_file), build=True, generate=True, verbose=False)

    return {
        "candidate_name": candidate["name"],
        "model_name": model.name,
        "model_upper": _model_upper(model.name),
        "generated_dir": str(generated_dir),
        "json_file": str(json_file),
        "runner_template": "kinematic_bicycle_asset_main.c.in",
        "horizon": horizon,
        "dt": dt,
        "target_speed_mps": float(candidate.get("target_speed_mps", 0.0)),
        "wheelbase": wheelbase,
        "delta_max": delta_max,
        "defaults": {"steps": int(candidate["closed_loop_steps"])},
    }


def export_double_integrator_3d(candidate: dict, case_root: Path) -> dict:
    import casadi as ca
    from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

    dt = float(candidate["dt"])
    horizon = int(candidate["horizon"])

    x = ca.SX.sym("x")
    y = ca.SX.sym("y")
    z = ca.SX.sym("z")
    vx = ca.SX.sym("vx")
    vy = ca.SX.sym("vy")
    vz = ca.SX.sym("vz")
    state = ca.vertcat(x, y, z, vx, vy, vz)

    ax = ca.SX.sym("ax")
    ay = ca.SX.sym("ay")
    az = ca.SX.sym("az")
    control = ca.vertcat(ax, ay, az)

    p = ca.SX.sym("p", 6)
    x_ref, y_ref, z_ref, vx_ref, vy_ref, vz_ref = [p[i] for i in range(6)]

    f_expl = ca.vertcat(vx, vy, vz, ax, ay, az)

    model = AcadosModel()
    model.name = "double_integrator_3d_asset"
    model.x = state
    model.xdot = ca.SX.sym("xdot", 6)
    model.u = control
    model.p = p
    model.f_expl_expr = f_expl
    model.f_impl_expr = model.xdot - f_expl
    model.cost_y_expr = ca.vertcat(
        x - x_ref, y - y_ref, z - z_ref, vx - vx_ref, vy - vy_ref, vz - vz_ref, ax, ay, az
    )
    model.cost_y_expr_e = ca.vertcat(x - x_ref, y - y_ref, z - z_ref, vx - vx_ref, vy - vy_ref, vz - vz_ref)

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = horizon
    ocp.parameter_values = np.zeros((6,))
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.W = np.diag([30.0, 30.0, 30.0, 5.0, 5.0, 5.0, 0.1, 0.1, 0.1])
    ocp.cost.W_e = np.diag([30.0, 30.0, 30.0, 5.0, 5.0, 5.0])
    ocp.cost.yref = np.zeros((9,))
    ocp.cost.yref_e = np.zeros((6,))
    ocp.constraints.x0 = np.zeros((6,))
    ocp.constraints.lbu = np.array([-15.0, -15.0, -15.0])
    ocp.constraints.ubu = np.array([15.0, 15.0, 15.0])
    ocp.constraints.idxbu = np.array([0, 1, 2], dtype=np.int64)
    ocp.solver_options.tf = horizon * dt
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_cond_N = horizon
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.tol = 1e-4
    ocp.solver_options.qp_tol = 1e-4

    generated_dir = case_root / "generated"
    json_file = case_root / "acados_ocp_double_integrator_3d_asset.json"
    ocp.code_export_directory = str(generated_dir)
    AcadosOcpSolver(ocp, json_file=str(json_file), build=True, generate=True, verbose=False)

    return {
        "candidate_name": candidate["name"],
        "model_name": model.name,
        "model_upper": _model_upper(model.name),
        "generated_dir": str(generated_dir),
        "json_file": str(json_file),
        "runner_template": "double_integrator_3d_asset_main.c.in",
        "horizon": horizon,
        "dt": dt,
        "defaults": {"steps": int(candidate["closed_loop_steps"])},
    }


EXPORTERS = {
    "kinematic_bicycle_track_following": export_kinematic_bicycle,
    "double_integrator_3d_tracking": export_double_integrator_3d,
}


def render_runner(metadata: dict, template_dir: Path) -> str:
    text = (template_dir / metadata["runner_template"]).read_text(encoding="utf-8")
    delta_max = float(metadata.get("delta_max", 0.50))
    replacements = {
        "@CANDIDATE_NAME@": metadata["candidate_name"],
        "@MODEL_NAME@": metadata["model_name"],
        "@MODEL_UPPER@": metadata["model_upper"],
        "@STEPS_DEFAULT@": str(metadata["defaults"]["steps"]),
        "@DT@": f"{float(metadata['dt']):.17g}",
        "@TARGET_SPEED@": f"{float(metadata.get('target_speed_mps', 0.0)):.17g}",
        "@WHEELBASE@": f"{float(metadata.get('wheelbase', 0.33)):.17g}",
        "@DELTA_MAX@": f"{delta_max:.17g}",
        "@DELTA_MIN@": f"{-delta_max:.17g}",
    }
    for key, value in replacements.items():
        text = text.replace(key, value)
    return text


def build_generated_runner(metadata: dict, case_root: Path, skip_build: bool) -> Path:
    generated_dir = Path(metadata["generated_dir"])
    runner_path = generated_dir / f"main_{metadata['model_name']}.c"
    if not skip_build:
        runner_source = render_runner(metadata, Path(__file__).resolve().parent / "templates")
        runner_path.write_text(runner_source, encoding="utf-8")

        env = os.environ.copy()
        build_dir = case_root / "build"
        makefile_path = generated_dir / "Makefile"
        cmake_lists_path = generated_dir / "CMakeLists.txt"
        if makefile_path.exists():
            run_command(["make", "-C", str(generated_dir), "clean"], env=env)
            run_command(["make", "-C", str(generated_dir), "example", "-j"], env=env)
            return generated_dir / f"main_{metadata['model_name']}"
        if cmake_lists_path.exists():
            cmake_cmd = ["cmake", "-S", str(generated_dir), "-B", str(build_dir), "-DBUILD_EXAMPLE=ON"]
            if env.get("ACADOS_INCLUDE_PATH"):
                cmake_cmd.append(f"-DACADOS_INCLUDE_PATH={env['ACADOS_INCLUDE_PATH']}")
            if env.get("ACADOS_LIB_PATH"):
                cmake_cmd.append(f"-DACADOS_LIB_PATH={env['ACADOS_LIB_PATH']}")
            run_command(cmake_cmd, env=env)
            run_command(["cmake", "--build", str(build_dir), "-j"], env=env)
            return build_dir / f"main_{metadata['model_name']}"
        fail(f"generated code directory has neither Makefile nor CMakeLists.txt: {generated_dir}")
    return generated_dir / f"main_{metadata['model_name']}"


def main() -> int:
    args = parse_args()
    candidate = load_candidate(args.candidate)
    candidate_name = candidate["name"]
    case_root = args.output_root / candidate_name
    case_root.mkdir(parents=True, exist_ok=True)
    metadata_path = case_root / "metadata.json"

    # The compiled runners link against shared libs in the acados repo; make sure the
    # environment is configured even when skipping export/codegen.
    configure_acados_python(args.acados_repo)

    if not args.skip_export:
        import_python_dependencies()
        exporter = EXPORTERS.get(candidate["model_family"])
        if exporter is None:
            fail(f"unsupported model_family for acados asset backend: {candidate['model_family']}")
        metadata = exporter(candidate, case_root)
        dump_json(metadata_path, metadata)
    else:
        if not metadata_path.exists():
            fail(f"metadata missing, cannot skip export: {metadata_path}")
        metadata = load_json(metadata_path)

    binary = build_generated_runner(metadata, case_root, args.skip_build)
    if args.prepare_only:
        return 0
    if not binary.exists():
        fail(f"benchmark binary not found: {binary}")

    output_path = args.output or (args.results_root / f"{candidate_name}.csv")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    asset_path = ROOT / candidate["asset"]
    cmd = [str(binary), "--asset", str(asset_path), "--output", str(output_path)]
    if args.steps is not None:
        cmd += ["--steps", str(args.steps)]
    if "target_speed_mps" in candidate:
        cmd += ["--target-speed", str(candidate["target_speed_mps"])]
    run_command(cmd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
