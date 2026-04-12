#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "doubleintegrator3dregressionmodel.h"
#include "kinematicbicycleregressionmodel.h"
#include "minisolver/core/types.h"
#include "minisolver/solver/solver.h"
#include "json.hpp"

using json = nlohmann::json;
using namespace minisolver;

namespace fs = std::filesystem;
namespace {

constexpr int kMaxHorizon = 256;

struct DriverArg {
    bool list = false;
    fs::path scenario_path;
    bool check_gt = false;
    fs::path scenario_root = "scenarios";
};

struct Scenario {
    std::string name;
    std::string domain;
    std::string model;
    double dt = 0.1;
    int N = 50;
    std::vector<double> x0;
    fs::path ref_path;
    fs::path gt_path;
    fs::path source_path;
};

struct GtReference {
    double objective = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> terminal_state;
    std::vector<double> first_control;
    std::string status;
    std::string solver;
    double tolerance = std::numeric_limits<double>::quiet_NaN();
};

struct RunResult {
    std::string status;
    int iters = -1;
    double time_ms = 0.0;
    double objective = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> terminal_state;
    std::vector<double> first_control;
};

bool gt_is_valid(const GtReference& gt) {
    // CasADi+IPOPT uses these return strings; keep this permissive but not misleading.
    return gt.status == "Solve_Succeeded" || gt.status == "Solved_To_Acceptable_Level";
}

struct DrivingRef {
    double t = 0.0;
    double x_ref = 0.0;
    double y_ref = 0.0;
    double yaw_ref = 0.0;
    double v_ref = 0.0;
    double w_left = 1.85;
    double w_right = 1.85;
};

struct RobotRef {
    double t = 0.0;
    double x_ref = 0.0;
    double y_ref = 0.0;
    double z_ref = 0.0;
    double vx_ref = 0.0;
    double vy_ref = 0.0;
    double vz_ref = 0.0;
};

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> out;
    out.reserve(8);
    std::string token;
    for (const char ch : line) {
        if (ch == ',') {
            out.push_back(token);
            token.clear();
        } else if (ch != '\r') {
            token.push_back(ch);
        }
    }
    out.push_back(token);
    for (auto& item : out) {
        while (!item.empty() && (item.back() == '\n' || item.back() == ' ' || item.back() == '\t'))
            item.pop_back();
        size_t begin = 0;
        while (begin < item.size() && (item[begin] == ' ' || item[begin] == '\t')) {
            ++begin;
        }
        if (begin > 0) {
            item = item.substr(begin);
        }
    }
    return out;
}

double wrap_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

std::vector<double> parse_numeric_row(const std::vector<std::string>& parts, const std::unordered_map<std::string, size_t>& idx_map, const std::string& key) {
    const auto it = idx_map.find(key);
    if (it == idx_map.end() || it->second >= parts.size()) {
        throw std::runtime_error("ref.csv missing column: " + key);
    }
    return {std::stod(parts[it->second])};
}

double get_value(const std::vector<std::string>& parts, const std::unordered_map<std::string, size_t>& idx_map, const std::string& key) {
    return parse_numeric_row(parts, idx_map, key).front();
}

std::vector<DrivingRef> load_driving_ref_csv(const fs::path& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("unable to open ref csv: " + path.string());
    }
    std::string header_line;
    std::getline(file, header_line);
    const auto headers = split_csv_line(header_line);
    std::unordered_map<std::string, size_t> idx;
    for (size_t i = 0; i < headers.size(); ++i) {
        idx[headers[i]] = i;
    }

    const std::array<std::string, 7> required = {
        "t", "x_ref", "y_ref", "yaw_ref", "v_ref", "w_left", "w_right"
    };
    for (const auto& key : required) {
        if (idx.find(key) == idx.end()) {
            throw std::runtime_error("driving ref missing required column: " + key);
        }
    }

    std::vector<DrivingRef> refs;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        const auto parts = split_csv_line(line);
        if (parts.empty()) continue;
        DrivingRef row;
        row.t = get_value(parts, idx, "t");
        row.x_ref = get_value(parts, idx, "x_ref");
        row.y_ref = get_value(parts, idx, "y_ref");
        row.yaw_ref = get_value(parts, idx, "yaw_ref");
        row.v_ref = get_value(parts, idx, "v_ref");
        row.w_left = get_value(parts, idx, "w_left");
        row.w_right = get_value(parts, idx, "w_right");
        refs.push_back(row);
    }
    return refs;
}

std::vector<RobotRef> load_robot_ref_csv(const fs::path& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("unable to open ref csv: " + path.string());
    }
    std::string header_line;
    std::getline(file, header_line);
    const auto headers = split_csv_line(header_line);
    std::unordered_map<std::string, size_t> idx;
    for (size_t i = 0; i < headers.size(); ++i) {
        idx[headers[i]] = i;
    }

    const std::array<std::string, 7> required = {
        "t", "x_ref", "y_ref", "z_ref", "vx_ref", "vy_ref", "vz_ref"
    };
    for (const auto& key : required) {
        if (idx.find(key) == idx.end()) {
            throw std::runtime_error("robotics ref missing required column: " + key);
        }
    }

    std::vector<RobotRef> refs;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        const auto parts = split_csv_line(line);
        if (parts.empty()) continue;
        RobotRef row;
        row.t = get_value(parts, idx, "t");
        row.x_ref = get_value(parts, idx, "x_ref");
        row.y_ref = get_value(parts, idx, "y_ref");
        row.z_ref = get_value(parts, idx, "z_ref");
        row.vx_ref = get_value(parts, idx, "vx_ref");
        row.vy_ref = get_value(parts, idx, "vy_ref");
        row.vz_ref = get_value(parts, idx, "vz_ref");
        refs.push_back(row);
    }
    return refs;
}

SolverConfig make_solver_config() {
    SolverConfig config;
    config.print_level = PrintLevel::NONE;
    config.integrator = IntegratorType::RK4_EXPLICIT;
    config.barrier_strategy = BarrierStrategy::MEHROTRA;
    config.line_search_type = LineSearchType::FILTER;
    config.enable_soc = false;
    config.enable_feasibility_restoration = true;
    config.enable_slack_reset = true;
    config.max_iters = 80;
    config.tol_con = 1e-3;
    config.tol_dual = 1e-3;
    config.backend = Backend::CPU_SERIAL;
    return config;
}

RunResult run_driving(const Scenario& scenario, const std::vector<DrivingRef>& refs_in) {
    RunResult out;
    const int horizon = std::min(std::min(scenario.N, static_cast<int>(refs_in.size()) - 1), kMaxHorizon);
    if (scenario.x0.size() != 5 || horizon < 1) {
        throw std::runtime_error("kinematic scenario requires N>=1 and x0 length 5");
    }

    MiniSolver<KinematicBicycleRegressionModel, kMaxHorizon> solver(horizon, Backend::CPU_SERIAL, make_solver_config());
    solver.set_dt(scenario.dt);
    solver.set_initial_state(scenario.x0);

    const auto refs = std::vector<DrivingRef>(refs_in.begin(), refs_in.begin() + horizon + 1);
    for (int k = 0; k <= horizon; ++k) {
        const auto& r = refs[static_cast<size_t>(k)];
        const double nx = -std::sin(r.yaw_ref);
        const double ny = std::cos(r.yaw_ref);
        solver.set_parameter(k, 0, r.x_ref);
        solver.set_parameter(k, 1, r.y_ref);
        solver.set_parameter(k, 2, r.yaw_ref);
        solver.set_parameter(k, 3, r.v_ref);
        solver.set_parameter(k, 4, nx);
        solver.set_parameter(k, 5, ny);
        solver.set_parameter(k, 6, r.w_left);
        solver.set_parameter(k, 7, r.w_right);

        solver.set_state_guess(k, 0, r.x_ref);
        solver.set_state_guess(k, 1, r.y_ref);
        solver.set_state_guess(k, 2, r.yaw_ref);
        solver.set_state_guess(k, 3, r.v_ref);
        if (k == 0) {
            solver.set_state_guess(k, 4, scenario.x0[4]);
        } else {
            solver.set_state_guess(k, 4, scenario.x0[4]);
        }
    }
    for (int k = 0; k < horizon; ++k) {
        solver.set_control_guess(k, 0, 0.0);
        solver.set_control_guess(k, 1, 0.0);
    }

    const auto t0 = std::chrono::steady_clock::now();
    const SolverStatus status = solver.solve();
    const auto t1 = std::chrono::steady_clock::now();
    out.status = status_to_string(status);
    out.iters = solver.get_iteration_count();
    out.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    out.objective = 0.0;
    for (int k = 0; k <= horizon; ++k) {
        out.objective += solver.get_stage_cost(k);
    }
    out.terminal_state = solver.get_state(horizon);
    out.first_control = solver.get_control(0);
    if (out.first_control.empty()) {
        out.first_control = {0.0, 0.0};
    }
    return out;
}

RunResult run_robotics(const Scenario& scenario, const std::vector<RobotRef>& refs_in) {
    RunResult out;
    const int horizon = std::min(std::min(scenario.N, static_cast<int>(refs_in.size()) - 1), kMaxHorizon);
    if (scenario.x0.size() != 6 || horizon < 1) {
        throw std::runtime_error("double integrator scenario requires N>=1 and x0 length 6");
    }

    MiniSolver<DoubleIntegrator3DRegressionModel, kMaxHorizon> solver(horizon, Backend::CPU_SERIAL, make_solver_config());
    solver.set_dt(scenario.dt);
    solver.set_initial_state(scenario.x0);

    const auto refs = std::vector<RobotRef>(refs_in.begin(), refs_in.begin() + horizon + 1);
    for (int k = 0; k <= horizon; ++k) {
        const auto& r = refs[static_cast<size_t>(k)];
        solver.set_parameter(k, 0, r.x_ref);
        solver.set_parameter(k, 1, r.y_ref);
        solver.set_parameter(k, 2, r.z_ref);
        solver.set_parameter(k, 3, r.vx_ref);
        solver.set_parameter(k, 4, r.vy_ref);
        solver.set_parameter(k, 5, r.vz_ref);

        solver.set_state_guess(k, 0, r.x_ref);
        solver.set_state_guess(k, 1, r.y_ref);
        solver.set_state_guess(k, 2, r.z_ref);
        solver.set_state_guess(k, 3, r.vx_ref);
        solver.set_state_guess(k, 4, r.vy_ref);
        solver.set_state_guess(k, 5, r.vz_ref);
    }

    for (int k = 0; k < horizon; ++k) {
        solver.set_control_guess(k, 0, 0.0);
        solver.set_control_guess(k, 1, 0.0);
        solver.set_control_guess(k, 2, 0.0);
    }

    const auto t0 = std::chrono::steady_clock::now();
    const SolverStatus status = solver.solve();
    const auto t1 = std::chrono::steady_clock::now();
    out.status = status_to_string(status);
    out.iters = solver.get_iteration_count();
    out.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    out.objective = 0.0;
    for (int k = 0; k <= horizon; ++k) {
        out.objective += solver.get_stage_cost(k);
    }
    out.terminal_state = solver.get_state(horizon);
    out.first_control = solver.get_control(0);
    if (out.first_control.empty()) {
        out.first_control = {0.0, 0.0, 0.0};
    }
    return out;
}

double vector_l2(const std::vector<double>& a, const std::vector<double>& b) {
    const size_t n = std::min(a.size(), b.size());
    if (n == 0) {
        return 0.0;
    }
    double sum = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double d = a[i] - b[i];
        sum += d * d;
    }
    return std::sqrt(sum);
}

bool is_json_file(const fs::path& p) {
    return p.extension() == ".json";
}

std::optional<GtReference> load_gt_reference(const fs::path& path) {
    if (path.empty() || !fs::exists(path)) {
        return std::nullopt;
    }
    std::ifstream file(path);
    if (!file.is_open()) {
        return std::nullopt;
    }
    json j;
    file >> j;
    if (!j.is_object()) {
        return std::nullopt;
    }

    GtReference gt;
    if (j.contains("objective")) {
        gt.objective = j.at("objective").get<double>();
    }
    if (j.contains("terminal_state")) {
        gt.terminal_state = j.at("terminal_state").get<std::vector<double>>();
    }
    if (j.contains("first_control")) {
        gt.first_control = j.at("first_control").get<std::vector<double>>();
    }
    if (j.contains("status")) {
        gt.status = j.at("status").get<std::string>();
    }
    if (j.contains("solver")) {
        gt.solver = j.at("solver").get<std::string>();
    }
    if (j.contains("tolerance")) {
        gt.tolerance = j.at("tolerance").get<double>();
    }
    return gt;
}

Scenario load_scenario(const fs::path& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("unable to open scenario json: " + path.string());
    }
    json j;
    file >> j;
    if (!j.is_object()) {
        throw std::runtime_error("scenario json is not an object");
    }

    Scenario scenario;
    scenario.source_path = fs::weakly_canonical(path);
    scenario.name = j.value("name", path.parent_path().filename().string());
    scenario.domain = j.value("domain", "");
    scenario.model = j.value("model", "");
    scenario.dt = j.value("dt", 0.1);
    scenario.N = j.value("N", 50);
    scenario.x0 = j.value("x0", std::vector<double>{});

    const fs::path base_dir = path.parent_path();
    const std::string ref = j.value("ref", std::string());
    if (ref.empty()) {
        throw std::runtime_error("scenario json missing 'ref'");
    }
    scenario.ref_path = base_dir / ref;

    if (j.contains("gt")) {
        const std::string gt = j.at("gt").get<std::string>();
        scenario.gt_path = base_dir / gt;
    }
    return scenario;
}

DriverArg parse_args(int argc, char** argv) {
    DriverArg args;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto take = [&](int i0, const char* name) -> std::string {
            if (i0 + 1 >= argc) {
                throw std::runtime_error(std::string("missing value for ") + name);
            }
            return argv[++i];
        };
        if (key == "--list") {
            args.list = true;
        } else if (key == "--scenario") {
            args.scenario_path = take(i, "--scenario");
        } else if (key == "--check-gt") {
            args.check_gt = true;
        } else if (key == "--scenario-dir") {
            args.scenario_root = take(i, "--scenario-dir");
        } else if (key == "--help" || key == "-h") {
            throw std::runtime_error("help");
        } else {
            throw std::runtime_error("unknown argument: " + key);
        }
    }
    return args;
}

json run_list(const fs::path& scenario_root) {
    json out = json::array();
    if (!fs::exists(scenario_root)) {
        return out;
    }

    for (const auto& entry : fs::directory_iterator(scenario_root)) {
        if (!entry.is_directory()) continue;
        const fs::path candidate = entry.path() / "scenario.json";
        if (!fs::exists(candidate) || !is_json_file(candidate)) continue;
        try {
            const Scenario scenario = load_scenario(candidate);
            out.push_back({
                {"name", scenario.name},
                {"path", fs::relative(candidate, fs::current_path()).string()},
                {"domain", scenario.domain},
                {"model", scenario.model},
                {"N", scenario.N},
                {"dt", scenario.dt},
            });
        } catch (...) {
            continue;
        }
    }
    return out;
}

json run_scenario(const fs::path& scenario_path, bool check_gt) {
    const Scenario scenario = load_scenario(scenario_path);
    const std::string model = scenario.model;
    const fs::path gt_path = scenario.gt_path.empty() ? (scenario_path.parent_path() / "gt.json") : scenario.gt_path;

    RunResult result;
    double gt_error = std::numeric_limits<double>::quiet_NaN();
    if (model == "kinematic_bicycle_v1") {
        auto refs = load_driving_ref_csv(scenario.ref_path);
        if (static_cast<int>(refs.size()) <= 1) {
            throw std::runtime_error("driving ref has too few rows");
        }
        result = run_driving(scenario, refs);
    } else if (model == "double_integrator_3d_v1") {
        auto refs = load_robot_ref_csv(scenario.ref_path);
        if (static_cast<int>(refs.size()) <= 1) {
            throw std::runtime_error("robotics ref has too few rows");
        }
        result = run_robotics(scenario, refs);
    } else {
        throw std::runtime_error("unsupported model id: " + model);
    }

    if (check_gt) {
        const auto gt = load_gt_reference(gt_path);
        if (gt && gt_is_valid(*gt)) {
            double obj_err = std::numeric_limits<double>::quiet_NaN();
            double terminal_err = std::numeric_limits<double>::quiet_NaN();
            double ctrl_err = std::numeric_limits<double>::quiet_NaN();
            if (std::isfinite(gt->objective) && std::isfinite(result.objective)) {
                obj_err = std::fabs(result.objective - gt->objective);
                if (std::isfinite(gt->objective) && std::fabs(gt->objective) > 1e-12) {
                    obj_err /= std::max(std::fabs(gt->objective), 1e-12);
                }
            }
            if (!gt->terminal_state.empty() && !result.terminal_state.empty()) {
                terminal_err = vector_l2(result.terminal_state, gt->terminal_state);
            }
            if (!gt->first_control.empty() && !result.first_control.empty()) {
                ctrl_err = vector_l2(result.first_control, gt->first_control);
            }
            gt_error = 0.0;
            if (std::isfinite(obj_err)) gt_error = std::max(gt_error, obj_err);
            if (std::isfinite(terminal_err)) gt_error = std::max(gt_error, terminal_err);
            if (std::isfinite(ctrl_err)) gt_error = std::max(gt_error, ctrl_err);
        }
    }

    json payload{
        {"status", result.status},
        {"time_ms", result.time_ms},
        {"iters", result.iters},
        {"objective", result.objective},
        {"gt_error", gt_error},
        {"first_control", result.first_control},
        {"terminal_state", result.terminal_state},
    };
    payload["scenario"] = scenario.name;
    payload["model"] = scenario.model;
    payload["domain"] = scenario.domain;
    return payload;
}

void print_usage() {
    std::cout << "Usage:\n"
              << "  nmpc_bench_minisolver --list [--scenario-dir PATH]\n"
              << "  nmpc_bench_minisolver --scenario PATH [--check-gt]\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const DriverArg args = parse_args(argc, argv);
        if (args.scenario_path.empty() && !args.list) {
            print_usage();
            return 2;
        }
        if (args.list) {
            const auto payload = run_list(args.scenario_root);
            std::cout << payload.dump() << "\n";
            return 0;
        }

        fs::path scenario_path = args.scenario_path;
        if (scenario_path.extension() != ".json") {
            scenario_path /= "scenario.json";
        }
        const auto payload = run_scenario(scenario_path, args.check_gt);
        std::cout << payload.dump() << "\n";
        return 0;
    } catch (const std::runtime_error& exc) {
        if (std::string(exc.what()) == "help") {
            print_usage();
            return 0;
        }
        std::cerr << exc.what() << "\n";
        print_usage();
        return 1;
    } catch (const std::exception& exc) {
        std::cerr << "runner error: " << exc.what() << "\n";
        return 1;
    }
}
