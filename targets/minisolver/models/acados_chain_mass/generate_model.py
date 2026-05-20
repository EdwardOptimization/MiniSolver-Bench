"""
Native MiniSolver SymPy model for the official acados disturbed chain-mass case.

Mirrors acados/examples/acados_python/chain_mass/export_disturbed_chain_mass_model.py
so that the MiniSolver benchmark runs on its own fused Riccati / codegen pipeline,
not on hand-written dense Jacobians.

State layout (n_mass = 5 → M = 3):
    x[ 0.. 2]  position of intermediate mass 0  (px0, py0, pz0)
    x[ 3.. 5]  position of intermediate mass 1  (px1, py1, pz1)
    x[ 6.. 8]  position of intermediate mass 2  (px2, py2, pz2)
    x[ 9..11]  position of free-end mass        (px3, py3, pz3)
    x[12..14]  velocity of intermediate mass 0
    x[15..17]  velocity of intermediate mass 1
    x[18..20]  velocity of intermediate mass 2

Controls u[0..2] are the velocity of the free-end mass (acados layout).

Parameters (NP = 9 + 21 + 21 + 3 = 54):
    p[ 0.. 8]  disturbance force on intermediate masses (3 per mass)
    p[ 9..29]  state reference  x_ref
    p[30..50]  state weight     W_x (diagonal)
    p[51..53]  control weight   W_u (diagonal)

Constraints (NC = 10):
    6 hard control bounds  |u_i| <= 1
    4 soft wall constraints  y_pos_wall - pos_y_i <= 0  (L2, weight 1e3)

Physics constants match the hand-written model in official_nmpc_benchmark.cpp:
    mass m = 0.033, spring D = 1.0, rest length L = 0.033, y_pos_wall = -0.05.
"""

import os
import sys

import sympy as sp

MINISOLVER_SOURCE_DIR = os.environ.get(
    "MINISOLVER_SOURCE_DIR",
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../MiniSolver")),
)
sys.path.append(os.path.join(MINISOLVER_SOURCE_DIR, "python"))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")))

from minisolver.MiniModel import OptimalControlModel  # noqa: E402
from targets.minisolver.models.common import set_dynamics  # noqa: E402


# =============================================================================
# Custom SymPy function: Euclidean norm with regularizer.
#
# sp.sqrt(a^2 + b^2 + c^2 + eps) triggers pathological radical simplification
# inside SymPy's .diff(), making Jacobian computation intractable for NX >= 15.
#
# This class keeps the norm opaque during differentiation: SymPy calls fdiff()
# which returns the exact chain-rule term d/da = a/norm without ever expanding
# the sqrt internals.  ccode() emits sqrt(...) for the C++ generator.
# =============================================================================
_NORM_EPS = 1e-12


class _Norm3(sp.Function):
    """sqrt(a^2 + b^2 + c^2 + eps) with efficient symbolic differentiation."""
    nargs = 3

    @classmethod
    def eval(cls, a, b, c):
        # Return None → keep symbolic (don't auto-evaluate)
        return None

    def fdiff(self, argindex):
        a, b, c = self.args
        return self.args[argindex - 1] / self  # d/da_i = a_i / norm

    def _ccode(self, printer):
        a, b, c = (printer._print(arg) for arg in self.args)
        return f"sqrt({a}*{a} + {b}*{b} + {c}*{c} + {_NORM_EPS})"

    # Make sp.ccode dispatch to our custom printer method
    def _sympystr(self, printer):
        return f"_Norm3({', '.join(str(a) for a in self.args)})"


# SymPy's default C-code printer needs a hook for custom functions.
# Register via the printing module so sp.ccode() works.
from sympy.printing.c import C99CodePrinter  # noqa: E402

_orig_ccode = C99CodePrinter._print_Function


def _ccode_with_norm3(self, expr):
    if isinstance(expr, _Norm3):
        a, b, c = (self._print(arg) for arg in expr.args)
        return f"sqrt({a}*{a} + {b}*{b} + {c}*{c} + {_NORM_EPS})"
    return _orig_ccode(self, expr)


C99CodePrinter._print_Function = _ccode_with_norm3


# --- Problem dimensions (matching official acados n_mass=5 case) ---
N_MASS = 5
M = N_MASS - 2                 # intermediate masses
POS_DIM = (M + 1) * 3          # intermediate (M) + free-end (1)
VEL_DIM = M * 3
NX = POS_DIM + VEL_DIM         # 21
NU = 3
NP_DIST = M * 3                # 9
NP_XREF = NX                   # 21
NP_WX = NX                     # 21
NP_WU = NU                     # 3
NP = NP_DIST + NP_XREF + NP_WX + NP_WU  # 54

# Physics constants (match official_nmpc_benchmark.cpp hand-written model)
MASS = 0.033
SPRING = 1.0
REST_LENGTH = 0.033
Y_POS_WALL = -0.05
GRAVITY = 9.81


def build_model() -> OptimalControlModel:
    model = OptimalControlModel("ChainMassModel")

    # States: position then velocity. Use short identifiers that are valid C++.
    pos_syms = []  # flat list length POS_DIM
    for i in range(M + 1):
        for axis in "xyz":
            pos_syms.append(model.state(f"p{axis}{i}"))

    vel_syms = []  # flat list length VEL_DIM (intermediate only)
    for i in range(M):
        for axis in "xyz":
            vel_syms.append(model.state(f"v{axis}{i}"))

    # Controls: velocity of the free-end mass
    ctrl_syms = [model.control(f"u{axis}") for axis in "xyz"]

    # Parameters: disturbance, state reference, state weights, control weights
    dist_syms = []
    for i in range(M):
        for axis in "xyz":
            dist_syms.append(model.parameter(f"w{axis}{i}"))

    xref_syms = [model.parameter(f"xref{i}") for i in range(NX)]
    wx_syms = [model.parameter(f"wstate{i}") for i in range(NX)]
    wu_syms = [model.parameter(f"wctrl{i}") for i in range(NU)]

    # Convenience views over positions: pos[i] is a 3-vector for mass i (i in 0..M).
    def mass_pos(i):
        return [pos_syms[3 * i + axis] for axis in range(3)]

    # --- Dynamics ---
    # Compute force on each intermediate mass: gravity + disturbance, then add spring forces.
    # force[i] is a 3-vector for mass i (i in 0..M-1).
    force = [[sp.Integer(0), sp.Integer(0), sp.Integer(0)] for _ in range(M)]
    for i in range(M):
        force[i][0] = dist_syms[3 * i + 0]
        force[i][1] = dist_syms[3 * i + 1]
        force[i][2] = -GRAVITY + dist_syms[3 * i + 2]

    # Spring loop over segments 0..M (wall-mass0, mass0-mass1, ..., mass(M-1)-massM).
    k_over_m = SPRING / MASS
    for seg in range(M + 1):
        right = mass_pos(seg)
        if seg == 0:
            dvec = list(right)  # wall at origin → dist = right
        else:
            left = mass_pos(seg - 1)
            dvec = [right[a] - left[a] for a in range(3)]

        norm = _Norm3(dvec[0], dvec[1], dvec[2])
        scale = k_over_m * (1 - REST_LENGTH / norm)
        F = [scale * d for d in dvec]

        # Mass on the right (index seg): force[seg] -= F  (only if intermediate, i.e. seg < M)
        if seg < M:
            for a in range(3):
                force[seg][a] = force[seg][a] - F[a]
        # Mass on the left (index seg-1): force[seg-1] += F  (only if seg > 0)
        if seg > 0:
            for a in range(3):
                force[seg - 1][a] = force[seg - 1][a] + F[a]

    # Wire up state derivatives.
    # Position dynamics:
    #   intermediate mass i has dpos_i/dt = vel_i   (for i in 0..M-1)
    #   free-end mass (index M) has dpos_M/dt = u
    for i in range(M):
        for axis in range(3):
            set_dynamics(model, pos_syms[3 * i + axis], vel_syms[3 * i + axis])
    for axis in range(3):
        set_dynamics(model, pos_syms[3 * M + axis], ctrl_syms[axis])

    # Velocity dynamics for intermediate masses: dvel_i/dt = force[i]
    for i in range(M):
        for axis in range(3):
            set_dynamics(model, vel_syms[3 * i + axis], force[i][axis])

    # --- Cost (weighted least squares on state + control) ---
    # cost = 0.5 * sum Wx_i * (x_i - xref_i)^2 + 0.5 * sum Wu_j * u_j^2
    state_syms_flat = pos_syms + vel_syms
    for i in range(NX):
        diff = state_syms_flat[i] - xref_syms[i]
        model.minimize(sp.Rational(1, 2) * wx_syms[i] * diff * diff)
    for j in range(NU):
        model.minimize(sp.Rational(1, 2) * wu_syms[j] * ctrl_syms[j] * ctrl_syms[j])

    # --- Constraints ---
    # Hard: |u_j| <= 1 for j = 0..2
    for j in range(NU):
        model.subject_to(ctrl_syms[j] - 1 <= 0)
        model.subject_to(-ctrl_syms[j] - 1 <= 0)

    # Soft L2, weight 1e3: y_pos_wall - pos_y_i <= 0, for each mass (i = 0..M including free end)
    for i in range(M + 1):
        model.subject_to(
            Y_POS_WALL - pos_syms[3 * i + 1] <= 0,
            weight=1e3,
            loss="L2",
        )

    return model


if __name__ == "__main__":
    import time

    t0 = time.time()
    print("Building symbolic model...", flush=True)
    model = build_model()
    print(f"  build_model done in {time.time() - t0:.1f}s", flush=True)

    nx = len(model.states)
    nu = len(model.controls)
    np_param = len(model.parameters)
    nc = len(model.constraints)
    print(f"  dims: NX={nx} NU={nu} NP={np_param} NC={nc}", flush=True)
    assert nx == NX, f"Expected NX={NX}, got {nx}"
    assert nu == NU, f"Expected NU={NU}, got {nu}"
    assert np_param == NP, f"Expected NP={NP}, got {np_param}"
    assert nc == 10, f"Expected NC=10, got {nc}"

    output_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../../generated")
    )
    # Fused Riccati symbolic expansion is O(NX^3) over a dense Vxx and explodes
    # for NX=21 (minutes of SymPy time + large generated kernel). The chain-mass
    # problem is better served by the runtime sparse Riccati path, so disable
    # the fused kernel here.
    t1 = time.time()
    print("Generating C++ model header...", flush=True)
    model.generate(output_dir, use_fused_riccati=False)
    print(f"  generate done in {time.time() - t1:.1f}s", flush=True)
    print(f"Generated ChainMassModel in {output_dir}")
