"""Idempotent postprocessor for checked-in official-case generated headers.

The base headers still contain the full ppoly tables. This pass changes the
model contract to consume callback-updated local Taylor jets through parameters,
matching MiniSolver's dcol callback pattern without hand-editing generated code.
"""

from __future__ import annotations

from pathlib import Path


def _replace_once(text: str, old: str, new: str, label: str) -> str:
    if old not in text:
        raise RuntimeError(f"could not find {label} anchor")
    return text.replace(old, new, 1)


def _insert_after_once(text: str, anchor: str, block: str, label: str) -> str:
    if anchor not in text:
        raise RuntimeError(f"could not find {label} insertion anchor")
    return text.replace(anchor, anchor + block, 1)


def ensure_race_callback_header(header_path: Path) -> None:
    if not header_path.exists():
        raise FileNotFoundError(f"missing generated base header: {header_path}")
    text = header_path.read_text(encoding="utf-8")

    if "P_CALLBACK_KAPPA_D3" not in text:
        text = _replace_once(
            text,
            "    static const int NP=16;\n",
            "    static const int NP=21;\n"
            "    static const int P_CALLBACK_TRACK_S_LIN=16;\n"
            "    static const int P_CALLBACK_KAPPA=17;\n"
            "    static const int P_CALLBACK_KAPPA_D1=18;\n"
            "    static const int P_CALLBACK_KAPPA_D2=19;\n"
            "    static const int P_CALLBACK_KAPPA_D3=20;\n",
            "race callback parameter constants",
        )

    if "callback_race_kappa(const MSVec<T, NP>& p" not in text:
        text = _insert_after_once(
            text,
            "    template <typename T>\n"
            "    static inline T ppoly_race_kappa_d3(const T& s) {\n"
            "        return ppoly_eval<T, 594, 0>(s, ppoly_race_kappa_d3_breaks, ppoly_race_kappa_d3_coeffs);\n"
            "    }\n\n",
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
            "    }\n\n",
            "race callback helper",
        )

    if '"track_s_lin"' not in text:
        text = _replace_once(
            text,
            '        "wu1",\n'
            "    };\n",
            '        "wu1",\n'
            '        "track_s_lin",\n'
            '        "race_kappa",\n'
            '        "race_kappa_d1",\n'
            '        "race_kappa_d2",\n'
            '        "race_kappa_d3",\n'
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

    if "P_CALLBACK_TRACK_Z_D1" not in text:
        text = _replace_once(
            text,
            "    static const int NP=48;\n",
            "    static const int NP=64;\n"
            "    static const int P_CALLBACK_TRACK_S_LIN=48;\n"
            "    static const int P_CALLBACK_TRACK_X_D1=49;\n"
            "    static const int P_CALLBACK_TRACK_Y_D1=54;\n"
            "    static const int P_CALLBACK_TRACK_Z_D1=59;\n",
            "quad callback parameter constants",
        )

    if "callback_quad_track_derivative(" not in text:
        text = _insert_after_once(
            text,
            "    template <typename T>\n"
            "    static inline T ppoly_quad_track_z_d5(const T& s) {\n"
            "        return ppoly_eval<T, 315, 0>(s, ppoly_quad_track_z_d5_breaks, ppoly_quad_track_z_d5_coeffs);\n"
            "    }\n\n",
            "    template <typename T>\n"
            "    static inline T callback_quad_track_derivative(\n"
            "        const MSVec<T, NP>& p, const T& s, int base_idx, int order)\n"
            "    {\n"
            "        const T ds = s - p(P_CALLBACK_TRACK_S_LIN);\n"
            "        const T d1 = p(base_idx + 0);\n"
            "        const T d2 = p(base_idx + 1);\n"
            "        const T d3 = p(base_idx + 2);\n"
            "        const T d4 = p(base_idx + 3);\n"
            "        const T d5 = p(base_idx + 4);\n"
            "        switch (order) {\n"
            "            case 1:\n"
            "                return d1 + d2 * ds + T(0.5) * d3 * ds * ds\n"
            "                    + T(1.0 / 6.0) * d4 * ds * ds * ds\n"
            "                    + T(1.0 / 24.0) * d5 * ds * ds * ds * ds;\n"
            "            case 2:\n"
            "                return d2 + d3 * ds + T(0.5) * d4 * ds * ds\n"
            "                    + T(1.0 / 6.0) * d5 * ds * ds * ds;\n"
            "            case 3:\n"
            "                return d3 + d4 * ds + T(0.5) * d5 * ds * ds;\n"
            "            case 4:\n"
            "                return d4 + d5 * ds;\n"
            "            default:\n"
            "                return d5;\n"
            "        }\n"
            "    }\n\n",
            "quad callback helper",
        )

    if '"quad_track_z_d5"' not in text:
        text = _replace_once(
            text,
            '        "wu3",\n'
            "    };\n",
            '        "wu3",\n'
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
            "    };\n",
            "quad callback parameter names",
        )

    continuous_old = "        T alpha4 = u_in(3);\n        (void)p_in;\n"
    if "P_CALLBACK_TRACK_X_D1, 5" not in text:
        text = _replace_once(
            text,
            continuous_old,
            "        T alpha4 = u_in(3);\n"
            "        const auto ppoly_quad_track_x_d1 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_D1, 1); };\n"
            "        const auto ppoly_quad_track_x_d2 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_D1, 2); };\n"
            "        const auto ppoly_quad_track_x_d3 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_D1, 3); };\n"
            "        const auto ppoly_quad_track_x_d4 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_D1, 4); };\n"
            "        const auto ppoly_quad_track_x_d5 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_D1, 5); };\n"
            "        const auto ppoly_quad_track_y_d1 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_D1, 1); };\n"
            "        const auto ppoly_quad_track_y_d2 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_D1, 2); };\n"
            "        const auto ppoly_quad_track_y_d3 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_D1, 3); };\n"
            "        const auto ppoly_quad_track_y_d4 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_D1, 4); };\n"
            "        const auto ppoly_quad_track_y_d5 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_D1, 5); };\n"
            "        const auto ppoly_quad_track_z_d1 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_D1, 1); };\n"
            "        const auto ppoly_quad_track_z_d2 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_D1, 2); };\n"
            "        const auto ppoly_quad_track_z_d3 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_D1, 3); };\n"
            "        const auto ppoly_quad_track_z_d4 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_D1, 4); };\n"
            "        const auto ppoly_quad_track_z_d5 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_D1, 5); };\n",
            "quad continuous callback lambdas",
        )
    if text.count(continuous_old) > 0:
        text = _replace_once(
            text,
            continuous_old,
            "        T alpha4 = u_in(3);\n"
            "        const auto ppoly_quad_track_x_d1 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_D1, 1); };\n"
            "        const auto ppoly_quad_track_x_d2 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_D1, 2); };\n"
            "        const auto ppoly_quad_track_x_d3 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_D1, 3); };\n"
            "        const auto ppoly_quad_track_x_d4 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_X_D1, 4); };\n"
            "        const auto ppoly_quad_track_y_d1 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_D1, 1); };\n"
            "        const auto ppoly_quad_track_y_d2 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_D1, 2); };\n"
            "        const auto ppoly_quad_track_y_d3 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_D1, 3); };\n"
            "        const auto ppoly_quad_track_y_d4 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Y_D1, 4); };\n"
            "        const auto ppoly_quad_track_z_d1 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_D1, 1); };\n"
            "        const auto ppoly_quad_track_z_d2 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_D1, 2); };\n"
            "        const auto ppoly_quad_track_z_d3 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_D1, 3); };\n"
            "        const auto ppoly_quad_track_z_d4 = [&](const T& value) { return callback_quad_track_derivative(p_in, value, P_CALLBACK_TRACK_Z_D1, 4); };\n",
            "quad integrate callback lambdas",
        )

    constraint_old = "        T s = kp.x(0);\n        T n = kp.x(1);\n\n"
    if "callback_quad_track_derivative(kp.p" not in text:
        text = _replace_once(
            text,
            constraint_old,
            "        T s = kp.x(0);\n"
            "        T n = kp.x(1);\n"
            "        const auto ppoly_quad_track_x_d2 = [&](const T& value) { return callback_quad_track_derivative(kp.p, value, P_CALLBACK_TRACK_X_D1, 2); };\n"
            "        const auto ppoly_quad_track_x_d3 = [&](const T& value) { return callback_quad_track_derivative(kp.p, value, P_CALLBACK_TRACK_X_D1, 3); };\n"
            "        const auto ppoly_quad_track_y_d2 = [&](const T& value) { return callback_quad_track_derivative(kp.p, value, P_CALLBACK_TRACK_Y_D1, 2); };\n"
            "        const auto ppoly_quad_track_y_d3 = [&](const T& value) { return callback_quad_track_derivative(kp.p, value, P_CALLBACK_TRACK_Y_D1, 3); };\n"
            "        const auto ppoly_quad_track_z_d2 = [&](const T& value) { return callback_quad_track_derivative(kp.p, value, P_CALLBACK_TRACK_Z_D1, 2); };\n"
            "        const auto ppoly_quad_track_z_d3 = [&](const T& value) { return callback_quad_track_derivative(kp.p, value, P_CALLBACK_TRACK_Z_D1, 3); };\n\n",
            "quad constraint callback lambdas",
        )

    header_path.write_text(text, encoding="utf-8")
