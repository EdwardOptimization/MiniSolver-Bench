"""Idempotent postprocessor for checked-in official-case generated headers.

The base headers still contain the full ppoly tables. This pass changes the
model contract to consume callback-updated track data through parameters,
matching MiniSolver's dcol callback pattern without hand-editing generated code.
"""

from __future__ import annotations

from pathlib import Path

RACE_CALLBACK_WINDOW_SEGMENTS = 9
QUAD_CALLBACK_WINDOW_SEGMENTS = 9


def _replace_once(text: str, old: str, new: str, label: str) -> str:
    if old not in text:
        raise RuntimeError(f"could not find {label} anchor")
    return text.replace(old, new, 1)


def _insert_after_once(text: str, anchor: str, block: str, label: str) -> str:
    if anchor not in text:
        raise RuntimeError(f"could not find {label} insertion anchor")
    return text.replace(anchor, anchor + block, 1)


def _race_callback_param_names() -> str:
    lines: list[str] = []
    for idx in range(RACE_CALLBACK_WINDOW_SEGMENTS + 1):
        lines.append(f'        "race_kappa_break_{idx}",\n')
    for segment in range(RACE_CALLBACK_WINDOW_SEGMENTS):
        for coeff in range(4):
            lines.append(f'        "race_kappa_c{coeff}_{segment}",\n')
    return "".join(lines)


def _quad_callback_param_names() -> str:
    lines: list[str] = []
    for axis in ("x", "y", "z"):
        for idx in range(QUAD_CALLBACK_WINDOW_SEGMENTS + 1):
            lines.append(f'        "quad_track_{axis}_break_{idx}",\n')
        for segment in range(QUAD_CALLBACK_WINDOW_SEGMENTS):
            for coeff in range(6):
                lines.append(f'        "quad_track_{axis}_c{coeff}_{segment}",\n')
    return "".join(lines)


def _ensure_race_soft_constraint_contract(text: str) -> str:
    if "constraint_has_l1" in text and "update_soft_constraint_weights" in text:
        return text

    soft_contract = (
        "    static constexpr std::array<bool, NC> constraint_has_l1 = "
        "{true, true, true, true, true, true, true, true, true, true, false, false, false, false};\n"
        "    static constexpr std::array<bool, NC> constraint_has_l2 = "
        "{false, false, false, false, false, false, false, false, false, false, false, false, false, false};\n"
        "    static constexpr bool any_l1_constraints = true;\n"
        "    static constexpr bool any_l2_constraints = false;\n\n"
        "    template <typename T>\n"
        "    static void update_soft_constraint_weights(KnotPoint<T,NX,NU,NC,NP>& kp) {\n"
        "        kp.l1_weight.setZero();\n"
        "        kp.l2_weight.setZero();\n"
        "        kp.l1_weight(0) = T(100.0);\n"
        "        kp.l1_weight(1) = T(100.0);\n"
        "        kp.l1_weight(2) = T(100.0);\n"
        "        kp.l1_weight(3) = T(100.0);\n"
        "        kp.l1_weight(4) = T(100.0);\n"
        "        kp.l1_weight(5) = T(100.0);\n"
        "        kp.l1_weight(6) = T(100.0);\n"
        "        kp.l1_weight(7) = T(100.0);\n"
        "        kp.l1_weight(8) = T(100.0);\n"
        "        kp.l1_weight(9) = T(100.0);\n"
        "    }\n\n"
    )
    anchor = (
        "    static constexpr std::array<int, NC> constraint_types = "
        "{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0};\n"
    )
    return _insert_after_once(text, anchor, soft_contract, "race soft constraint contract")


def ensure_race_callback_header(header_path: Path) -> None:
    if not header_path.exists():
        raise FileNotFoundError(f"missing generated base header: {header_path}")
    text = header_path.read_text(encoding="utf-8")
    text = _ensure_race_soft_constraint_contract(text)

    race_window_np = 16 + (RACE_CALLBACK_WINDOW_SEGMENTS + 1) + RACE_CALLBACK_WINDOW_SEGMENTS * 4
    race_window_constants = (
        f"    static const int NP={race_window_np};\n"
        f"    static const int P_CALLBACK_WINDOW_SEGMENTS={RACE_CALLBACK_WINDOW_SEGMENTS};\n"
        "    static const int P_CALLBACK_BREAKS=16;\n"
        "    static const int P_CALLBACK_COEFFS=P_CALLBACK_BREAKS + P_CALLBACK_WINDOW_SEGMENTS + 1;\n"
    )
    race_taylor_constants = (
        "    static const int NP=21;\n"
        "    static const int P_CALLBACK_TRACK_S_LIN=16;\n"
        "    static const int P_CALLBACK_KAPPA=17;\n"
        "    static const int P_CALLBACK_KAPPA_D1=18;\n"
        "    static const int P_CALLBACK_KAPPA_D2=19;\n"
        "    static const int P_CALLBACK_KAPPA_D3=20;\n"
    )
    if "P_CALLBACK_WINDOW_SEGMENTS" not in text:
        if race_taylor_constants in text:
            text = text.replace(race_taylor_constants, race_window_constants, 1)
        else:
            text = _replace_once(
                text,
                "    static const int NP=16;\n",
                race_window_constants,
                "race callback parameter constants",
            )

    race_window_helper = (
        "    template <typename T>\n"
        "    static inline int callback_race_kappa_segment(const MSVec<T, NP>& p, const T& s) {\n"
        "        const double sd = static_cast<double>(s);\n"
        "        if (sd < static_cast<double>(p(P_CALLBACK_BREAKS))) {\n"
        "            return 0;\n"
        "        }\n"
        "        if (sd >= static_cast<double>(p(P_CALLBACK_BREAKS + P_CALLBACK_WINDOW_SEGMENTS))) {\n"
        "            return P_CALLBACK_WINDOW_SEGMENTS - 1;\n"
        "        }\n"
        "        for (int i = 0; i < P_CALLBACK_WINDOW_SEGMENTS; ++i) {\n"
        "            if (sd < static_cast<double>(p(P_CALLBACK_BREAKS + i + 1))) {\n"
        "                return i;\n"
        "            }\n"
        "        }\n"
        "        return P_CALLBACK_WINDOW_SEGMENTS - 1;\n"
        "    }\n\n"
        "    static inline int callback_race_kappa_coeff_offset(int segment) {\n"
        "        return P_CALLBACK_COEFFS + 4 * segment;\n"
        "    }\n\n"
        "    template <typename T>\n"
        "    static inline T callback_race_kappa(const MSVec<T, NP>& p, const T& s) {\n"
        "        const int segment = callback_race_kappa_segment(p, s);\n"
        "        const int offset = callback_race_kappa_coeff_offset(segment);\n"
        "        const T ds = s - p(P_CALLBACK_BREAKS + segment);\n"
        "        T y = p(offset + 0);\n"
        "        y = y * ds + p(offset + 1);\n"
        "        y = y * ds + p(offset + 2);\n"
        "        y = y * ds + p(offset + 3);\n"
        "        return y;\n"
        "    }\n\n"
        "    template <typename T>\n"
        "    static inline T callback_race_kappa_d1(const MSVec<T, NP>& p, const T& s) {\n"
        "        const int segment = callback_race_kappa_segment(p, s);\n"
        "        const int offset = callback_race_kappa_coeff_offset(segment);\n"
        "        const T ds = s - p(P_CALLBACK_BREAKS + segment);\n"
        "        return (T(3.0) * p(offset + 0) * ds + T(2.0) * p(offset + 1)) * ds + p(offset + 2);\n"
        "    }\n\n"
    )
    race_taylor_helper = (
        "    template <typename T>\n"
        "    static inline T callback_race_kappa(const MSVec<T, NP>& p, const T& s) {\n"
        "        const T ds = s - p(P_CALLBACK_TRACK_S_LIN);\n"
        "        return p(P_CALLBACK_KAPPA)\n"
        "            + p(P_CALLBACK_KAPPA_D1) * ds\n"
        "            + T(0.5) * p(P_CALLBACK_KAPPA_D2) * ds * ds\n"
        "            + T(1.0 / 6.0) * p(P_CALLBACK_KAPPA_D3) * ds * ds * ds;\n"
        "    }\n\n"
        "    template <typename T>\n"
        "    static inline T callback_race_kappa_d1(const MSVec<T, NP>& p, const T& s) {\n"
        "        const T ds = s - p(P_CALLBACK_TRACK_S_LIN);\n"
        "        return p(P_CALLBACK_KAPPA_D1)\n"
        "            + p(P_CALLBACK_KAPPA_D2) * ds\n"
        "            + T(0.5) * p(P_CALLBACK_KAPPA_D3) * ds * ds;\n"
        "    }\n\n"
    )
    if "callback_race_kappa_segment" not in text:
        if race_taylor_helper in text:
            text = text.replace(race_taylor_helper, race_window_helper, 1)
        else:
            text = _insert_after_once(
                text,
                "    template <typename T>\n"
                "    static inline T ppoly_race_kappa_d3(const T& s) {\n"
                "        return ppoly_eval<T, 594, 0>(s, ppoly_race_kappa_d3_breaks, ppoly_race_kappa_d3_coeffs);\n"
                "    }\n\n",
                race_window_helper,
                "race callback helper",
            )
    text = text.replace(
        "        if (sd < static_cast<double>(p(P_CALLBACK_BREAKS))\n"
        "            || sd >= static_cast<double>(p(P_CALLBACK_BREAKS + P_CALLBACK_WINDOW_SEGMENTS))) {\n"
        "            return -1;\n"
        "        }\n",
        "        if (sd < static_cast<double>(p(P_CALLBACK_BREAKS))) {\n"
        "            return 0;\n"
        "        }\n"
        "        if (sd >= static_cast<double>(p(P_CALLBACK_BREAKS + P_CALLBACK_WINDOW_SEGMENTS))) {\n"
        "            return P_CALLBACK_WINDOW_SEGMENTS - 1;\n"
        "        }\n",
    )
    text = text.replace(
        "        if (segment < 0) {\n"
        "            return ppoly_race_kappa(s);\n"
        "        }\n",
        "",
    )
    text = text.replace(
        "        if (segment < 0) {\n"
        "            return ppoly_race_kappa_d1(s);\n"
        "        }\n",
        "",
    )
    text = text.replace(
        "                kp.f_resid = rk4_step<T>(kp.x, kp.u, kp.p, dt, &kp.A, &kp.B);\n"
        "                break;\n",
        "                MSVec<T, NX> x_sub = kp.x;\n"
        "                MSMat<T, NX, NX> A_total;\n"
        "                MSMat<T, NX, NU> B_total;\n"
        "                MatOps::setIdentity(A_total);\n"
        "                B_total.setZero();\n"
        "                const double sub_dt = dt / 3.0;\n"
        "                for (int substep = 0; substep < 3; ++substep) {\n"
        "                    MSMat<T, NX, NX> A_step;\n"
        "                    MSMat<T, NX, NU> B_step;\n"
        "                    x_sub = rk4_step<T>(x_sub, kp.u, kp.p, sub_dt, &A_step, &B_step);\n"
        "                    B_total = A_step * B_total + B_step;\n"
        "                    A_total = A_step * A_total;\n"
        "                }\n"
        "                kp.f_resid = x_sub;\n"
        "                kp.A = A_total;\n"
        "                kp.B = B_total;\n"
        "                break;\n",
    )

    old_param_names = (
        '        "track_s_lin",\n'
        '        "race_kappa",\n'
        '        "race_kappa_d1",\n'
        '        "race_kappa_d2",\n'
        '        "race_kappa_d3",\n'
    )
    if '"race_kappa_break_0"' not in text:
        if old_param_names in text:
            text = text.replace(old_param_names, _race_callback_param_names(), 1)
        else:
            text = _replace_once(
                text,
                '        "wu1",\n'
                "    };\n",
                '        "wu1",\n'
                f"{_race_callback_param_names()}"
                "    };\n",
                "race callback parameter names",
            )

    continuous_old = "        T derDelta = u_in(1);\n        (void)p_in;\n"
    if "const auto ppoly_race_kappa = [&](const T& value) { return callback_race_kappa(p_in, value); };" not in text:
        text = _replace_once(
            text,
            continuous_old,
            "        T derDelta = u_in(1);\n"
            "        const auto ppoly_race_kappa = [&](const T& value) { return callback_race_kappa(p_in, value); };\n"
            "        const auto ppoly_race_kappa_d1 = [&](const T& value) { return callback_race_kappa_d1(p_in, value); };\n",
            "race continuous callback lambdas",
        )
    if text.count(continuous_old) > 0:
        text = _replace_once(
            text,
            continuous_old,
            "        T derDelta = u_in(1);\n"
            "        const auto ppoly_race_kappa = [&](const T& value) { return callback_race_kappa(p_in, value); };\n",
            "race integrate callback lambda",
        )

    header_path.write_text(text, encoding="utf-8")


def ensure_quad_callback_header(header_path: Path) -> None:
    if not header_path.exists():
        raise FileNotFoundError(f"missing generated base header: {header_path}")
    text = header_path.read_text(encoding="utf-8")

    axis_stride = (QUAD_CALLBACK_WINDOW_SEGMENTS + 1) + QUAD_CALLBACK_WINDOW_SEGMENTS * 6
    quad_window_np = 48 + 3 * axis_stride
    quad_window_constants = (
        f"    static const int NP={quad_window_np};\n"
        f"    static const int P_CALLBACK_WINDOW_SEGMENTS={QUAD_CALLBACK_WINDOW_SEGMENTS};\n"
        f"    static const int P_CALLBACK_AXIS_STRIDE={axis_stride};\n"
        "    static const int P_CALLBACK_TRACK_X_BREAKS=48;\n"
        "    static const int P_CALLBACK_TRACK_X_COEFFS=P_CALLBACK_TRACK_X_BREAKS + P_CALLBACK_WINDOW_SEGMENTS + 1;\n"
        "    static const int P_CALLBACK_TRACK_Y_BREAKS=P_CALLBACK_TRACK_X_BREAKS + P_CALLBACK_AXIS_STRIDE;\n"
        "    static const int P_CALLBACK_TRACK_Y_COEFFS=P_CALLBACK_TRACK_Y_BREAKS + P_CALLBACK_WINDOW_SEGMENTS + 1;\n"
        "    static const int P_CALLBACK_TRACK_Z_BREAKS=P_CALLBACK_TRACK_Y_BREAKS + P_CALLBACK_AXIS_STRIDE;\n"
        "    static const int P_CALLBACK_TRACK_Z_COEFFS=P_CALLBACK_TRACK_Z_BREAKS + P_CALLBACK_WINDOW_SEGMENTS + 1;\n"
    )
    quad_taylor_constants = (
        "    static const int NP=64;\n"
        "    static const int P_CALLBACK_TRACK_S_LIN=48;\n"
        "    static const int P_CALLBACK_TRACK_X_D1=49;\n"
        "    static const int P_CALLBACK_TRACK_Y_D1=54;\n"
        "    static const int P_CALLBACK_TRACK_Z_D1=59;\n"
    )
    if "P_CALLBACK_AXIS_STRIDE" not in text:
        if quad_taylor_constants in text:
            text = text.replace(quad_taylor_constants, quad_window_constants, 1)
        else:
            text = _replace_once(
                text,
                "    static const int NP=48;\n",
                quad_window_constants,
                "quad callback parameter constants",
            )

    quad_window_helper = (
        "    template <typename T>\n"
        "    static inline int callback_quad_track_segment(const MSVec<T, NP>& p, const T& s, int break_base) {\n"
        "        const double sd = static_cast<double>(s);\n"
        "        if (sd < static_cast<double>(p(break_base))) {\n"
        "            return 0;\n"
        "        }\n"
        "        if (sd >= static_cast<double>(p(break_base + P_CALLBACK_WINDOW_SEGMENTS))) {\n"
        "            return P_CALLBACK_WINDOW_SEGMENTS - 1;\n"
        "        }\n"
        "        for (int i = 0; i < P_CALLBACK_WINDOW_SEGMENTS; ++i) {\n"
        "            if (sd < static_cast<double>(p(break_base + i + 1))) {\n"
        "                return i;\n"
        "            }\n"
        "        }\n"
        "        return P_CALLBACK_WINDOW_SEGMENTS - 1;\n"
        "    }\n\n"
        "    static inline double callback_quad_derivative_factor(int power, int order) {\n"
        "        double factor = 1.0;\n"
        "        for (int d = 0; d < order; ++d) {\n"
        "            factor *= static_cast<double>(power - d);\n"
        "        }\n"
        "        return factor;\n"
        "    }\n\n"
        "    template <typename T>\n"
        "    static inline T callback_quad_track_derivative(\n"
        "        const MSVec<T, NP>& p, const T& s, int break_base, int coeff_base, int order)\n"
        "    {\n"
        "        const int segment = callback_quad_track_segment(p, s, break_base);\n"
        "        const T ds = s - p(break_base + segment);\n"
        "        const int offset = coeff_base + 6 * segment;\n"
        "        T result = T(0.0);\n"
        "        for (int i = 0; i < 6; ++i) {\n"
        "            const int power = 5 - i;\n"
        "            if (power < order) {\n"
        "                continue;\n"
        "            }\n"
        "            T term = T(callback_quad_derivative_factor(power, order)) * p(offset + i);\n"
        "            for (int e = 0; e < power - order; ++e) {\n"
        "                term *= ds;\n"
        "            }\n"
        "            result += term;\n"
        "        }\n"
        "        return result;\n"
        "    }\n\n"
    )

    if "static inline int callback_quad_track_segment(" in text:
        start = text.index("    template <typename T>\n    static inline int callback_quad_track_segment(")
        end = text.index("\n\n    // --- Continuous Dynamics", start)
        text = text[:start] + quad_window_helper + text[end:]
    elif "static inline T callback_quad_track_derivative(" in text:
        start = text.index("    template <typename T>\n    static inline T callback_quad_track_derivative(")
        end = text.index("\n\n    // --- Continuous Dynamics", start)
        text = text[:start] + quad_window_helper + text[end:]
    elif "callback_quad_track_derivative(" not in text:
        text = _insert_after_once(
            text,
            "    template <typename T>\n"
            "    static inline T ppoly_quad_track_z_d5(const T& s) {\n"
            "        return ppoly_eval<T, 315, 0>(s, ppoly_quad_track_z_d5_breaks, ppoly_quad_track_z_d5_coeffs);\n"
            "    }\n\n",
            quad_window_helper,
            "quad callback helper",
        )

    old_param_names = (
        '        "track_s_lin",\n'
        '        "quad_track_x_d1",\n'
        '        "quad_track_x_d2",\n'
        '        "quad_track_x_d3",\n'
        '        "quad_track_x_d4",\n'
        '        "quad_track_x_d5",\n'
        '        "quad_track_y_d1",\n'
        '        "quad_track_y_d2",\n'
        '        "quad_track_y_d3",\n'
        '        "quad_track_y_d4",\n'
        '        "quad_track_y_d5",\n'
        '        "quad_track_z_d1",\n'
        '        "quad_track_z_d2",\n'
        '        "quad_track_z_d3",\n'
        '        "quad_track_z_d4",\n'
        '        "quad_track_z_d5",\n'
    )
    if '"quad_track_x_break_0"' not in text:
        if old_param_names in text:
            text = text.replace(old_param_names, _quad_callback_param_names(), 1)
        else:
            text = _replace_once(
                text,
                '        "wu3",\n'
                "    };\n",
                '        "wu3",\n'
                f"{_quad_callback_param_names()}"
                "    };\n",
                "quad callback parameter names",
            )

    old_bases = {
        "P_CALLBACK_TRACK_X_D1": ("P_CALLBACK_TRACK_X_BREAKS", "P_CALLBACK_TRACK_X_COEFFS"),
        "P_CALLBACK_TRACK_Y_D1": ("P_CALLBACK_TRACK_Y_BREAKS", "P_CALLBACK_TRACK_Y_COEFFS"),
        "P_CALLBACK_TRACK_Z_D1": ("P_CALLBACK_TRACK_Z_BREAKS", "P_CALLBACK_TRACK_Z_COEFFS"),
    }
    for old_base, (break_base, coeff_base) in old_bases.items():
        for order in range(1, 6):
            text = text.replace(
                f"callback_quad_track_derivative(p_in, value, {old_base}, {order})",
                f"callback_quad_track_derivative(p_in, value, {break_base}, {coeff_base}, {order})",
            )
            text = text.replace(
                f"callback_quad_track_derivative(kp.p, value, {old_base}, {order})",
                f"callback_quad_track_derivative(kp.p, value, {break_base}, {coeff_base}, {order})",
            )

    continuous_old = "        T alpha4 = u_in(3);\n        (void)p_in;\n"
    if "P_CALLBACK_TRACK_X_COEFFS, 5" not in text:
        text = _replace_once(
            text,
            continuous_old,
            "        T alpha4 = u_in(3);\n"
            "        const auto ppoly_quad_track_x_d1 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_BREAKS, P_CALLBACK_TRACK_X_COEFFS, 1); };\n"
            "        const auto ppoly_quad_track_x_d2 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_BREAKS, P_CALLBACK_TRACK_X_COEFFS, 2); };\n"
            "        const auto ppoly_quad_track_x_d3 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_BREAKS, P_CALLBACK_TRACK_X_COEFFS, 3); };\n"
            "        const auto ppoly_quad_track_x_d4 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_BREAKS, P_CALLBACK_TRACK_X_COEFFS, 4); };\n"
            "        const auto ppoly_quad_track_x_d5 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_BREAKS, P_CALLBACK_TRACK_X_COEFFS, 5); };\n"
            "        const auto ppoly_quad_track_y_d1 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_BREAKS, P_CALLBACK_TRACK_Y_COEFFS, 1); };\n"
            "        const auto ppoly_quad_track_y_d2 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_BREAKS, P_CALLBACK_TRACK_Y_COEFFS, 2); };\n"
            "        const auto ppoly_quad_track_y_d3 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_BREAKS, P_CALLBACK_TRACK_Y_COEFFS, 3); };\n"
            "        const auto ppoly_quad_track_y_d4 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_BREAKS, P_CALLBACK_TRACK_Y_COEFFS, 4); };\n"
            "        const auto ppoly_quad_track_y_d5 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_BREAKS, P_CALLBACK_TRACK_Y_COEFFS, 5); };\n"
            "        const auto ppoly_quad_track_z_d1 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_BREAKS, P_CALLBACK_TRACK_Z_COEFFS, 1); };\n"
            "        const auto ppoly_quad_track_z_d2 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_BREAKS, P_CALLBACK_TRACK_Z_COEFFS, 2); };\n"
            "        const auto ppoly_quad_track_z_d3 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_BREAKS, P_CALLBACK_TRACK_Z_COEFFS, 3); };\n"
            "        const auto ppoly_quad_track_z_d4 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_BREAKS, P_CALLBACK_TRACK_Z_COEFFS, 4); };\n"
            "        const auto ppoly_quad_track_z_d5 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_BREAKS, P_CALLBACK_TRACK_Z_COEFFS, 5); };\n",
            "quad continuous callback lambdas",
        )
    if text.count(continuous_old) > 0:
        text = _replace_once(
            text,
            continuous_old,
            "        T alpha4 = u_in(3);\n"
            "        const auto ppoly_quad_track_x_d1 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_BREAKS, P_CALLBACK_TRACK_X_COEFFS, 1); };\n"
            "        const auto ppoly_quad_track_x_d2 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_BREAKS, P_CALLBACK_TRACK_X_COEFFS, 2); };\n"
            "        const auto ppoly_quad_track_x_d3 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_BREAKS, P_CALLBACK_TRACK_X_COEFFS, 3); };\n"
            "        const auto ppoly_quad_track_x_d4 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_BREAKS, P_CALLBACK_TRACK_X_COEFFS, 4); };\n"
            "        const auto ppoly_quad_track_y_d1 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_BREAKS, P_CALLBACK_TRACK_Y_COEFFS, 1); };\n"
            "        const auto ppoly_quad_track_y_d2 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_BREAKS, P_CALLBACK_TRACK_Y_COEFFS, 2); };\n"
            "        const auto ppoly_quad_track_y_d3 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_BREAKS, P_CALLBACK_TRACK_Y_COEFFS, 3); };\n"
            "        const auto ppoly_quad_track_y_d4 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_BREAKS, P_CALLBACK_TRACK_Y_COEFFS, 4); };\n"
            "        const auto ppoly_quad_track_z_d1 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_BREAKS, P_CALLBACK_TRACK_Z_COEFFS, 1); };\n"
            "        const auto ppoly_quad_track_z_d2 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_BREAKS, P_CALLBACK_TRACK_Z_COEFFS, 2); };\n"
            "        const auto ppoly_quad_track_z_d3 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_BREAKS, P_CALLBACK_TRACK_Z_COEFFS, 3); };\n"
            "        const auto ppoly_quad_track_z_d4 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_BREAKS, P_CALLBACK_TRACK_Z_COEFFS, 4); };\n",
            "quad integrate callback lambdas",
        )

    constraint_old = "        T s = kp.x(0);\n        T n = kp.x(1);\n\n"
    if "callback_quad_track_derivative(kp.p" not in text:
        text = _replace_once(
            text,
            constraint_old,
            "        T s = kp.x(0);\n"
            "        T n = kp.x(1);\n"
            "        const auto ppoly_quad_track_x_d2 = [&](const T& value) { return callback_quad_track_derivative(kp.p, value, P_CALLBACK_TRACK_X_BREAKS, P_CALLBACK_TRACK_X_COEFFS, 2); };\n"
            "        const auto ppoly_quad_track_x_d3 = [&](const T& value) { return callback_quad_track_derivative(kp.p, value, P_CALLBACK_TRACK_X_BREAKS, P_CALLBACK_TRACK_X_COEFFS, 3); };\n"
            "        const auto ppoly_quad_track_y_d2 = [&](const T& value) { return callback_quad_track_derivative(kp.p, value, P_CALLBACK_TRACK_Y_BREAKS, P_CALLBACK_TRACK_Y_COEFFS, 2); };\n"
            "        const auto ppoly_quad_track_y_d3 = [&](const T& value) { return callback_quad_track_derivative(kp.p, value, P_CALLBACK_TRACK_Y_BREAKS, P_CALLBACK_TRACK_Y_COEFFS, 3); };\n"
            "        const auto ppoly_quad_track_z_d2 = [&](const T& value) { return callback_quad_track_derivative(kp.p, value, P_CALLBACK_TRACK_Z_BREAKS, P_CALLBACK_TRACK_Z_COEFFS, 2); };\n"
            "        const auto ppoly_quad_track_z_d3 = [&](const T& value) { return callback_quad_track_derivative(kp.p, value, P_CALLBACK_TRACK_Z_BREAKS, P_CALLBACK_TRACK_Z_COEFFS, 3); };\n\n",
            "quad constraint callback lambdas",
        )

    header_path.write_text(text, encoding="utf-8")
