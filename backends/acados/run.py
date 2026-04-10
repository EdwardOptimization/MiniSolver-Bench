#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ctypes
import json
import math
import os
import subprocess
import sys
from pathlib import Path


if __package__ in (None, ""):
    ROOT = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(ROOT))
    from cases import backend_case_names, load_case
else:
    ROOT = Path(__file__).resolve().parents[2]
    from ...cases import backend_case_names, load_case


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


def c_array(values: list[float]) -> str:
    return ", ".join(f"{float(v):.17g}" for v in values)


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def dump_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2, sort_keys=True)
        fh.write("\n")


def export_pendulum_on_cart(acados_repo: Path, case_root: Path) -> dict:
    import numpy as np
    import scipy.linalg
    from casadi import vertcat
    from acados_template import AcadosOcp, AcadosOcpSolver

    example_root = acados_repo / "examples" / "acados_python" / "pendulum_on_cart"
    sys.path.insert(0, str(example_root / "common"))
    from pendulum_model import export_pendulum_ode_model

    x0 = np.array([0.0, np.pi, 0.0, 0.0])
    fmax = 80.0
    tf = 0.8
    n_horizon = 40

    ocp = AcadosOcp()
    model = export_pendulum_ode_model()
    ocp.model = model

    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu
    ocp.solver_options.N_horizon = n_horizon

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    q_mat = 2 * np.diag([1e3, 1e3, 1e-2, 1e-2])
    r_mat = 2 * np.diag([1e-2])
    ocp.cost.W = scipy.linalg.block_diag(q_mat, r_mat)
    ocp.cost.W_e = q_mat
    ocp.model.cost_y_expr = vertcat(model.x, model.u)
    ocp.model.cost_y_expr_e = model.x
    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((nx,))

    ocp.constraints.lbu = np.array([-fmax])
    ocp.constraints.ubu = np.array([fmax])
    ocp.constraints.idxbu = np.array([0])
    ocp.constraints.x0 = x0

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "IRK"
    ocp.solver_options.sim_method_newton_iter = 10
    ocp.translate_nls_cost_to_conl()
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver_cond_N = n_horizon
    ocp.solver_options.tf = tf

    generated_dir = case_root / "generated"
    json_file = case_root / "acados_ocp_pendulum.json"
    ocp.code_export_directory = str(generated_dir)
    AcadosOcpSolver(ocp, json_file=str(json_file), build=True, generate=True, verbose=False)

    return {
        "case_name": "pendulum_on_cart",
        "model_name": model.name,
        "model_upper": model.name.upper(),
        "generated_dir": str(generated_dir),
        "json_file": str(json_file),
        "defaults": {"steps": 100},
        "x0_init": x0.tolist(),
        "tf": tf,
        "horizon": n_horizon,
    }


def export_race_cars(acados_repo: Path, case_root: Path) -> dict:
    import numpy as np
    import scipy.linalg
    from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

    example_root = acados_repo / "examples" / "acados_python" / "race_cars"
    sys.path.insert(0, str(example_root))
    from bicycle_model import bicycle_model

    tf = 1.0
    n_horizon = 50
    track_file = "LMS_Track.txt"

    ocp = AcadosOcp()
    model, constraint = bicycle_model(track_file)
    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.u
    model_ac.z = model.z
    model_ac.p = model.p
    model_ac.name = model.name
    model_ac.con_h_expr = constraint.expr
    ocp.model = model_ac

    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu
    ny_e = nx
    ocp.solver_options.N_horizon = n_horizon
    nh = int(constraint.expr.shape[0])
    nsbx = 1
    nsh = nh
    ns = nsbx + nsh

    q = np.diag([1e-1, 1e-8, 1e-8, 1e-8, 1e-3, 5e-3])
    r = np.eye(nu)
    r[0, 0] = 1e-3
    r[1, 1] = 5e-3
    q_e = np.diag([5e0, 1e1, 1e-8, 1e-8, 5e-3, 2e-3])

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    unscale = n_horizon / tf
    ocp.cost.W = unscale * scipy.linalg.block_diag(q, r)
    ocp.cost.W_e = q_e / unscale

    vx = np.zeros((ny, nx))
    vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = vx
    vu = np.zeros((ny, nu))
    vu[6, 0] = 1.0
    vu[7, 1] = 1.0
    ocp.cost.Vu = vu
    vx_e = np.zeros((ny_e, nx))
    vx_e[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = vx_e

    ocp.cost.zl = 100 * np.ones((ns,))
    ocp.cost.Zl = np.zeros((ns,))
    ocp.cost.zu = 100 * np.ones((ns,))
    ocp.cost.Zu = np.zeros((ns,))
    ocp.cost.yref = np.array([1, 0, 0, 0, 0, 0, 0, 0])
    ocp.cost.yref_e = np.array([0, 0, 0, 0, 0, 0])

    ocp.constraints.lbx = np.array([-12.0])
    ocp.constraints.ubx = np.array([12.0])
    ocp.constraints.idxbx = np.array([1])
    ocp.constraints.lbu = np.array([model.dthrottle_min, model.ddelta_min])
    ocp.constraints.ubu = np.array([model.dthrottle_max, model.ddelta_max])
    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lsbx = np.zeros((nsbx,))
    ocp.constraints.usbx = np.zeros((nsbx,))
    ocp.constraints.idxsbx = np.array([0])

    ocp.constraints.lh = np.array(
        [
            constraint.along_min,
            constraint.alat_min,
            model.n_min,
            model.throttle_min,
            model.delta_min,
        ]
    )
    ocp.constraints.uh = np.array(
        [
            constraint.along_max,
            constraint.alat_max,
            model.n_max,
            model.throttle_max,
            model.delta_max,
        ]
    )
    ocp.constraints.lsh = np.zeros(nsh)
    ocp.constraints.ush = np.zeros(nsh)
    ocp.constraints.idxsh = np.array(range(nsh))
    ocp.constraints.x0 = model.x0

    ocp.solver_options.tf = tf
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3

    generated_dir = case_root / "generated"
    json_file = case_root / "acados_ocp_race_cars.json"
    ocp.code_export_directory = str(generated_dir)
    AcadosOcpSolver(ocp, json_file=str(json_file), build=True, generate=True, verbose=False)

    return {
        "case_name": "race_cars",
        "model_name": model.name,
        "model_upper": model.name.upper(),
        "generated_dir": str(generated_dir),
        "json_file": str(json_file),
        "defaults": {"steps": int(10.0 * n_horizon / tf)},
        "x0_init": model.x0.tolist(),
        "tf": tf,
        "horizon": n_horizon,
        "track_pathlength": float(constraint.pathlength),
        "sref_n": 3.0,
    }


def export_chain_mass(acados_repo: Path, case_root: Path) -> dict:
    import numpy as np
    import scipy.linalg
    from acados_template import AcadosOcp, AcadosOcpSolver

    example_root = acados_repo / "examples" / "acados_python" / "chain_mass"
    sys.path.insert(0, str(example_root))
    from export_disturbed_chain_mass_model import export_disturbed_chain_mass_model
    from utils import compute_steady_state, get_chain_params

    chain_params = get_chain_params()
    n_mass = chain_params["n_mass"]
    m = chain_params["m"]
    d_spring = chain_params["D"]
    rest_length = chain_params["L"]
    m_intermediate = n_mass - 2
    ts = chain_params["Ts"]
    tf = chain_params["N"] * ts
    n_horizon = chain_params["N"]
    y_pos_wall = chain_params["yPosWall"]
    perturb_scale = chain_params["perturb_scale"]
    seed = chain_params["seed"]

    ocp = AcadosOcp()
    model = export_disturbed_chain_mass_model(n_mass, m, d_spring, rest_length)
    ocp.model = model

    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu

    x_pos_first_mass = np.zeros((3, 1))
    x_end_ref = np.zeros((3, 1))
    x_end_ref[0] = rest_length * (m_intermediate + 1) * 6
    xrest = compute_steady_state(n_mass, m, d_spring, rest_length, x_pos_first_mass, x_end_ref)

    ocp.solver_options.N_horizon = n_horizon
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    q_diag = np.ones((nx, 1))
    strong_penalty = m_intermediate + 1
    q_diag[3 * m_intermediate] = strong_penalty
    q_diag[3 * m_intermediate + 1] = strong_penalty
    q_diag[3 * m_intermediate + 2] = strong_penalty
    q = 2 * np.diagflat(q_diag)
    r = 2 * np.diagflat(1e-2 * np.ones((nu, 1)))

    ocp.cost.W = scipy.linalg.block_diag(q, r)
    ocp.cost.W_e = q
    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    vu = np.zeros((ny, nu))
    vu[nx:nx + nu, :] = np.eye(nu)
    ocp.cost.Vu = vu
    ocp.cost.Vx_e = np.eye(nx)

    yref = np.vstack((xrest, np.zeros((nu, 1)))).flatten()
    ocp.cost.yref = yref
    ocp.cost.yref_e = xrest.flatten()

    umax = np.ones((nu,))
    ocp.constraints.lbu = -umax
    ocp.constraints.ubu = umax
    ocp.constraints.idxbu = np.array(range(nu))
    ocp.constraints.x0 = xrest.reshape((nx,))
    ocp.parameter_values = np.zeros((3 * m_intermediate,))

    nbx = m_intermediate + 1
    jbx = np.zeros((nbx, nx))
    for i in range(nbx):
        jbx[i, 3 * i + 1] = 1.0
    ocp.constraints.Jbx = jbx
    ocp.constraints.lbx = y_pos_wall * np.ones((nbx,))
    ocp.constraints.ubx = 1e9 * np.ones((nbx,))
    ocp.constraints.Jsbx = np.eye(nbx)
    ocp.cost.Zl = 1e3 * np.ones((nbx,))
    ocp.cost.Zu = 1e3 * np.ones((nbx,))
    ocp.cost.zl = np.ones((nbx,))
    ocp.cost.zu = np.ones((nbx,))

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "IRK"
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.nlp_solver_max_iter = chain_params["nlp_iter"]
    ocp.solver_options.sim_method_num_stages = 2
    ocp.solver_options.sim_method_num_steps = 2
    ocp.solver_options.qp_solver_cond_N = n_horizon
    ocp.solver_options.qp_tol = chain_params["nlp_tol"]
    ocp.solver_options.tol = chain_params["nlp_tol"]
    ocp.solver_options.tf = tf

    generated_dir = case_root / "generated"
    json_file = case_root / "acados_ocp_chain_mass.json"
    ocp.code_export_directory = str(generated_dir)
    AcadosOcpSolver(ocp, json_file=str(json_file), build=True, generate=True, verbose=False)

    return {
        "case_name": "chain_mass",
        "model_name": model.name,
        "model_upper": model.name.upper(),
        "generated_dir": str(generated_dir),
        "json_file": str(json_file),
        "defaults": {"steps": int(math.floor(chain_params["Tsim"] / ts))},
        "xrest_init": xrest.flatten().tolist(),
        "u_init": chain_params["u_init"].tolist(),
        "y_pos_wall": float(y_pos_wall),
        "wall_count": int(nbx),
        "perturb_scale": float(perturb_scale),
        "seed": int(seed),
    }


def export_quadrotor_nav(acados_repo: Path, case_root: Path) -> dict:
    import numpy as np
    import scipy.linalg
    import casadi as ca
    from acados_template import ACADOS_INFTY, AcadosModel, AcadosOcp, AcadosOcpSolver

    example_root = acados_repo / "examples" / "acados_python" / "quadrotor_nav"
    sys.path.insert(0, str(example_root))
    import common
    from sys_dynamics import SysDyn

    sys_model = SysDyn()
    zeta_f, dyn_f, u, proj_constr, _dyn_fn = sys_model.SetupOde()

    ocp = AcadosOcp()
    model_ac = AcadosModel()
    model_ac.f_expl_expr = dyn_f
    model_ac.x = zeta_f
    model_ac.u = u
    model_ac.name = "drone_FrenSer"
    model_ac.con_h_expr = ca.vertcat(proj_constr)
    model_ac.con_h_expr_e = ca.vertcat(proj_constr)
    ocp.model = model_ac

    ocp.solver_options.N_horizon = common.N
    nx = int(model_ac.x.size()[0])
    nu = int(model_ac.u.size()[0])
    ocp.constraints.x0 = np.copy(common.init_zeta)

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.model.cost_y_expr = ca.vertcat(model_ac.x, model_ac.u)
    ocp.cost.yref = np.array(
        [
            0.2, 0, 0,
            1, 0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            common.U_HOV, common.U_HOV, common.U_HOV, common.U_HOV,
            0, 0, 0, 0,
        ]
    )
    ocp.cost.W = scipy.linalg.block_diag(common.Q, common.R)

    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.model.cost_y_expr_e = model_ac.x
    ocp.cost.yref_e = np.array(
        [
            0.2, 0, 0,
            1, 0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            common.U_HOV, common.U_HOV, common.U_HOV, common.U_HOV,
        ]
    )
    ocp.cost.W_e = common.Qn

    nh = int(model_ac.con_h_expr.shape[0])
    lh = np.full((nh,), -ACADOS_INFTY)
    uh = np.ones((nh,))
    ocp.constraints.lh = lh
    ocp.constraints.uh = uh
    ocp.constraints.lh_e = lh
    ocp.constraints.uh_e = uh

    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.tf = common.Tf
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.qp_solver_cond_N = int(common.N / 2)
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.tol = 1e-3

    generated_dir = case_root / "generated"
    json_file = case_root / "acados_ocp_quadrotor.json"
    ocp.code_export_directory = str(generated_dir)
    AcadosOcpSolver(ocp, json_file=str(json_file), build=True, generate=True, verbose=False)

    return {
        "case_name": "quadrotor_nav",
        "model_name": model_ac.name,
        "model_upper": model_ac.name.upper(),
        "generated_dir": str(generated_dir),
        "json_file": str(json_file),
        "defaults": {"steps": int(common.Nsim)},
        "init_zeta": common.init_zeta.tolist(),
        "u_hov": float(common.U_HOV),
        "s_ref": float(common.S_REF),
        "s_max": float(common.S_MAX),
        "tf": float(common.Tf),
        "horizon": int(common.N),
    }


EXPORTERS = {
    "pendulum_on_cart": export_pendulum_on_cart,
    "race_cars": export_race_cars,
    "chain_mass": export_chain_mass,
    "quadrotor_nav": export_quadrotor_nav,
}


def render_runner(metadata: dict, template_dir: Path) -> str:
    template_path = template_dir / metadata["runner_template"]
    text = template_path.read_text(encoding="utf-8")
    replacements = {
        "@CASE_NAME@": metadata["case_name"],
        "@MODEL_NAME@": metadata["model_name"],
        "@MODEL_UPPER@": metadata["model_upper"],
        "@STEPS_DEFAULT@": str(metadata["defaults"]["steps"]),
        "@X0_INIT@": c_array(metadata.get("x0_init", [])),
        "@HORIZON@": str(metadata.get("horizon", 0)),
        "@SREF_N@": f"{float(metadata.get('sref_n', 0.0)):.17g}",
        "@TRACK_PATHLENGTH@": f"{float(metadata.get('track_pathlength', 0.0)):.17g}",
        "@XREST_INIT@": c_array(metadata.get("xrest_init", [])),
        "@U_INIT@": c_array(metadata.get("u_init", [])),
        "@YPOS_WALL@": f"{float(metadata.get('y_pos_wall', 0.0)):.17g}",
        "@PERTURB_SCALE@": f"{float(metadata.get('perturb_scale', 0.0)):.17g}",
        "@SEED@": str(int(metadata.get("seed", 0))),
        "@WALL_COUNT@": str(int(metadata.get("wall_count", 0))),
        "@INIT_ZETA@": c_array(metadata.get("init_zeta", [])),
        "@U_HOV@": f"{float(metadata.get('u_hov', 0.0)):.17g}",
        "@S_REF@": f"{float(metadata.get('s_ref', 0.0)):.17g}",
        "@S_MAX@": f"{float(metadata.get('s_max', 0.0)):.17g}",
        "@TF@": f"{float(metadata.get('tf', 0.0)):.17g}",
    }
    for key, value in replacements.items():
        text = text.replace(key, value)
    return text


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
    parser = argparse.ArgumentParser(description="Export, build and run one official acados benchmark case.")
    parser.add_argument("--case", choices=backend_case_names("acados"), required=True)
    parser.add_argument("--acados-repo", type=Path, default=default_acados_repo())
    parser.add_argument("--output-root", type=Path, default=Path("out/acados"))
    parser.add_argument("--results-root", type=Path, default=Path("results/raw/acados"))
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--skip-export", action="store_true")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--prepare-only", action="store_true", help="Export/build assets but do not execute the benchmark binary.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    case_spec = load_case(args.case)
    case_root = args.output_root / args.case
    case_root.mkdir(parents=True, exist_ok=True)
    metadata_path = case_root / "metadata.json"

    if not args.skip_export:
        import_python_dependencies()
        configure_acados_python(args.acados_repo)
        metadata = EXPORTERS[args.case](args.acados_repo.resolve(), case_root)
        metadata["runner_template"] = case_spec["backends"]["acados"]["runner_template"]
        metadata["official_example"] = case_spec["official_example"]
        dump_json(metadata_path, metadata)
    else:
        if not metadata_path.exists():
            fail(f"metadata missing, cannot skip export: {metadata_path}")
        metadata = load_json(metadata_path)

    generated_dir = Path(metadata["generated_dir"])
    if not generated_dir.exists():
        fail(f"generated code directory not found: {generated_dir}")

    build_dir = case_root / "build"
    binary = generated_dir / f"main_{metadata['model_name']}"

    if not args.skip_build:
        template_dir = Path(__file__).resolve().parent / "templates"
        runner_source = render_runner(metadata, template_dir)
        runner_path = generated_dir / f"main_{metadata['model_name']}.c"
        runner_path.write_text(runner_source, encoding="utf-8")

        env = os.environ.copy()
        makefile_path = generated_dir / "Makefile"
        cmake_lists_path = generated_dir / "CMakeLists.txt"
        if makefile_path.exists():
            run_command(["make", "-C", str(generated_dir), "clean"], env=env)
            run_command(["make", "-C", str(generated_dir), "example", "-j"], env=env)
        elif cmake_lists_path.exists():
            cmake_cmd = ["cmake", "-S", str(generated_dir), "-B", str(build_dir), "-DBUILD_EXAMPLE=ON"]
            if env.get("ACADOS_INCLUDE_PATH"):
                cmake_cmd.append(f"-DACADOS_INCLUDE_PATH={env['ACADOS_INCLUDE_PATH']}")
            if env.get("ACADOS_LIB_PATH"):
                cmake_cmd.append(f"-DACADOS_LIB_PATH={env['ACADOS_LIB_PATH']}")
            run_command(cmake_cmd, env=env)
            run_command(["cmake", "--build", str(build_dir), "-j"], env=env)
            binary = build_dir / f"main_{metadata['model_name']}"
        else:
            fail(f"generated code directory has neither Makefile nor CMakeLists.txt: {generated_dir}")

    if args.prepare_only:
        return 0

    if not binary.exists():
        fail(f"benchmark binary not found: {binary}")

    output_path = args.output or (args.results_root / f"{args.case}.csv")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = [str(binary), "--output", str(output_path)]
    if args.steps is not None:
        cmd += ["--steps", str(args.steps)]
    run_command(cmd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
