#pragma once

// Experiment configuration for the STATIC paper reproduction (M8, 9 Nov 2026).
//
// One flat, text-based, strictly validated key/value format (INI-like: `key = value`, optional `[section]` prefixes, `#` or `;`
// comments). Every value an experiment can vary lives in ONE table (`config_fields()`), which drives parsing, command-line
// overrides and the canonical dump, so no default is duplicated: the defaults are the members' own default initialisers
// (tdr::config::*Config, tdr::search::VnsOptions, ShakeOptions, N4Options, KMeansPolicy). The resolved configuration is echoed
// into every result directory and parsing the echo reproduces the experiment (round trip is tested).
//
// Nothing here changes an algorithm default: a default-constructed ExperimentConfig runs exactly the M7 defaults.

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include "config/static_config.hpp"
#include "init/customer_clustering.hpp"
#include "search/vns.hpp"

namespace tdr::config {

// F1 (truck-only TSP) and F2 (k-means) constants that earlier milestones fixed inside the code (M1 follow-up). The defaults equal the
// values that were hard-coded before (TSPSolverFactory{}: 21 / 8 / 42; KMeansPolicy{}), so the initial solution is unchanged.
struct InitialConfig {
    int tsp_max_exact_n = 21;              // exact Held-Karp up to this many nodes
    int tsp_heuristic_restarts = 8;
    std::uint64_t tsp_heuristic_seed = 42; // NOT derived from seed.base_seed (kept as before M8; documented)
    tdr::init::KMeansPolicy kmeans;        // max_iterations, convergence_epsilon
};

struct ExperimentSection {
    std::string name = "static";
    int num_datasets = 1;                  // datasets dataset_index .. dataset_index + num_datasets - 1
    int runs = 10;                         // VNS runs per dataset ("10 runs" protocol of the paper; run indices first_run_index ..)
    std::uint64_t first_run_index = 0;
};

struct OutputSection {
    bool record_trace = false;             // per-pass trace of every run (traces.csv)
    bool write_solutions = true;           // best solution of every run in results.json
};

struct ExperimentConfig {
    StaticConfig base;                     // instance, vehicle, FLP, neighbourhood, seed.base_seed
    InitialConfig initial;
    tdr::search::VnsOptions vns;           // base_seed / run_index / n1,n2 repeats / record_trace are set by the runner from the sections around it
    ExperimentSection experiment;
    OutputSection output;
};

// ------------------------------------------------------------------------------------------------ the field table
struct ConfigField {
    std::string key;
    std::function<std::string(const ExperimentConfig&)> get;
    std::function<std::string(ExperimentConfig&, const std::string&)> set; // returns "" on success, else the problem
};

namespace detail_cfg {

inline std::string fmt_double(double v) {
    char buf[40];
    std::snprintf(buf, sizeof buf, "%.17g", v);
    return buf;
}

inline std::string trim(const std::string& s) {
    std::size_t a = 0, b = s.size();
    while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r')) ++a;
    while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r')) --b;
    return s.substr(a, b - a);
}

inline std::string parse_double(const std::string& v, double& out) {
    if (v.empty()) return "empty value";
    char* end = nullptr;
    errno = 0;
    const double d = std::strtod(v.c_str(), &end);
    if (end != v.c_str() + v.size() || errno == ERANGE) return "'" + v + "' is not a valid number";
    if (!std::isfinite(d)) return "'" + v + "' is not finite";
    out = d;
    return "";
}

inline std::string parse_i64(const std::string& v, long long& out) {
    if (v.empty()) return "empty value";
    char* end = nullptr;
    errno = 0;
    const long long d = std::strtoll(v.c_str(), &end, 10);
    if (end != v.c_str() + v.size() || errno == ERANGE) return "'" + v + "' is not a valid integer";
    out = d;
    return "";
}

inline std::string parse_u64(const std::string& v, unsigned long long& out) {
    if (v.empty()) return "empty value";
    if (v[0] == '-' || v[0] == '+') return "'" + v + "' is not a valid non-negative integer";
    char* end = nullptr;
    errno = 0;
    const unsigned long long d = std::strtoull(v.c_str(), &end, 10);
    if (end != v.c_str() + v.size() || errno == ERANGE) return "'" + v + "' is not a valid non-negative integer";
    out = d;
    return "";
}

inline std::string parse_bool(const std::string& v, bool& out) {
    if (v == "true" || v == "1") { out = true; return ""; }
    if (v == "false" || v == "0") { out = false; return ""; }
    return "'" + v + "' is not a boolean (true / false)";
}

inline std::string op_name(tdr::search::OperatorId o) {
    switch (o) {
        case tdr::search::OperatorId::N1: return "N1";
        case tdr::search::OperatorId::N2: return "N2";
        case tdr::search::OperatorId::N3: return "N3";
        case tdr::search::OperatorId::N4: return "N4";
        case tdr::search::OperatorId::N5: return "N5";
        case tdr::search::OperatorId::N6: return "N6";
        case tdr::search::OperatorId::N7: return "N7";
        case tdr::search::OperatorId::N8: return "N8";
    }
    return "?";
}

} // namespace detail_cfg

inline const std::vector<ConfigField>& config_fields() {
    using detail_cfg::fmt_double;
    using tdr::search::AcceptanceReference;
    using tdr::search::AffectedTripPolicy;
    using tdr::search::N4Mode;
    using tdr::search::OperatorId;
    static const std::vector<ConfigField> F = [] {
        std::vector<ConfigField> f;
        auto dbl = [&](const char* key, std::function<double&(ExperimentConfig&)> ref) {
            f.push_back({key, [ref](const ExperimentConfig& c) { return fmt_double(ref(const_cast<ExperimentConfig&>(c))); },
                         [ref](ExperimentConfig& c, const std::string& v) { double d = 0; std::string e = detail_cfg::parse_double(v, d); if (e.empty()) ref(c) = d; return e; }});
        };
        auto i32 = [&](const char* key, std::function<int&(ExperimentConfig&)> ref) {
            f.push_back({key, [ref](const ExperimentConfig& c) { return std::to_string(ref(const_cast<ExperimentConfig&>(c))); },
                         [ref](ExperimentConfig& c, const std::string& v) {
                             long long d = 0; std::string e = detail_cfg::parse_i64(v, d);
                             if (e.empty() && (d < -2147483647LL || d > 2147483647LL)) e = "'" + v + "' is out of range";
                             if (e.empty()) ref(c) = static_cast<int>(d);
                             return e; }});
        };
        auto u64 = [&](const char* key, std::function<std::uint64_t&(ExperimentConfig&)> ref) {
            f.push_back({key, [ref](const ExperimentConfig& c) { return std::to_string(ref(const_cast<ExperimentConfig&>(c))); },
                         [ref](ExperimentConfig& c, const std::string& v) { unsigned long long d = 0; std::string e = detail_cfg::parse_u64(v, d); if (e.empty()) ref(c) = static_cast<std::uint64_t>(d); return e; }});
        };
        auto usz = [&](const char* key, std::function<std::size_t&(ExperimentConfig&)> ref) {
            f.push_back({key, [ref](const ExperimentConfig& c) { return std::to_string(ref(const_cast<ExperimentConfig&>(c))); },
                         [ref](ExperimentConfig& c, const std::string& v) { unsigned long long d = 0; std::string e = detail_cfg::parse_u64(v, d); if (e.empty()) ref(c) = static_cast<std::size_t>(d); return e; }});
        };
        auto bl = [&](const char* key, std::function<bool&(ExperimentConfig&)> ref) {
            f.push_back({key, [ref](const ExperimentConfig& c) { return std::string(ref(const_cast<ExperimentConfig&>(c)) ? "true" : "false"); },
                         [ref](ExperimentConfig& c, const std::string& v) { bool b = false; std::string e = detail_cfg::parse_bool(v, b); if (e.empty()) ref(c) = b; return e; }});
        };

        f.push_back({"experiment.name", [](const ExperimentConfig& c) { return c.experiment.name; },
                     [](ExperimentConfig& c, const std::string& v) -> std::string { if (v.empty()) return "empty name"; c.experiment.name = v; return ""; }});
        i32("experiment.num_datasets", [](ExperimentConfig& c) -> int& { return c.experiment.num_datasets; });
        i32("experiment.runs", [](ExperimentConfig& c) -> int& { return c.experiment.runs; });
        u64("experiment.first_run_index", [](ExperimentConfig& c) -> std::uint64_t& { return c.experiment.first_run_index; });
        bl("output.record_trace", [](ExperimentConfig& c) -> bool& { return c.output.record_trace; });
        bl("output.write_solutions", [](ExperimentConfig& c) -> bool& { return c.output.write_solutions; });

        u64("seed.base_seed", [](ExperimentConfig& c) -> std::uint64_t& { return c.base.seed.base_seed; });

        i32("instance.num_customers", [](ExperimentConfig& c) -> int& { return c.base.instance.num_customers; });
        dbl("instance.area_side_m", [](ExperimentConfig& c) -> double& { return c.base.instance.area_side_m; });
        f.push_back({"instance.depot_placement",
                     [](const ExperimentConfig& c) { switch (c.base.instance.depot_placement) { case DepotPlacement::Centre: return std::string("centre"); case DepotPlacement::Corner: return std::string("corner"); case DepotPlacement::Explicit: return std::string("explicit"); } return std::string("centre"); },
                     [](ExperimentConfig& c, const std::string& v) -> std::string {
                         if (v == "centre") c.base.instance.depot_placement = DepotPlacement::Centre;
                         else if (v == "corner") c.base.instance.depot_placement = DepotPlacement::Corner;
                         else if (v == "explicit") c.base.instance.depot_placement = DepotPlacement::Explicit;
                         else return "'" + v + "' is not one of centre, corner, explicit";
                         return ""; }});
        dbl("instance.depot_x_m", [](ExperimentConfig& c) -> double& { return c.base.instance.depot_x_m; });
        dbl("instance.depot_y_m", [](ExperimentConfig& c) -> double& { return c.base.instance.depot_y_m; });
        i32("instance.dataset_index", [](ExperimentConfig& c) -> int& { return c.base.instance.dataset_index; });
        dbl("instance.truck_service_time_s", [](ExperimentConfig& c) -> double& { return c.base.instance.truck_service_time_s; });
        dbl("instance.drone_service_time_s", [](ExperimentConfig& c) -> double& { return c.base.instance.drone_service_time_s; });
        dbl("instance.theta", [](ExperimentConfig& c) -> double& { return c.base.instance.theta; });

        dbl("vehicle.truck_speed_mps", [](ExperimentConfig& c) -> double& { return c.base.vehicle.truck_speed_mps; });
        dbl("vehicle.drone_speed_mps", [](ExperimentConfig& c) -> double& { return c.base.vehicle.drone_speed_mps; });
        dbl("vehicle.endurance_s", [](ExperimentConfig& c) -> double& { return c.base.vehicle.endurance_s; });
        dbl("vehicle.launch_prep_time_s", [](ExperimentConfig& c) -> double& { return c.base.vehicle.launch_prep_time_s; });
        dbl("vehicle.recovery_time_s", [](ExperimentConfig& c) -> double& { return c.base.vehicle.recovery_time_s; });
        i32("vehicle.num_drones", [](ExperimentConfig& c) -> int& { return c.base.vehicle.num_drones; });

        dbl("flp.unit_division_length_m", [](ExperimentConfig& c) -> double& { return c.base.flp.unit_division_length_m; });

        i32("neighbourhood.adjacency_window", [](ExperimentConfig& c) -> int& { return c.base.neighbourhood.adjacency_window; });
        i32("neighbourhood.n1_repeats", [](ExperimentConfig& c) -> int& { return c.base.neighbourhood.n1_repeats; });
        i32("neighbourhood.n2_repeats", [](ExperimentConfig& c) -> int& { return c.base.neighbourhood.n2_repeats; });

        i32("initial.tsp_max_exact_n", [](ExperimentConfig& c) -> int& { return c.initial.tsp_max_exact_n; });
        i32("initial.tsp_heuristic_restarts", [](ExperimentConfig& c) -> int& { return c.initial.tsp_heuristic_restarts; });
        u64("initial.tsp_heuristic_seed", [](ExperimentConfig& c) -> std::uint64_t& { return c.initial.tsp_heuristic_seed; });
        i32("initial.kmeans_max_iterations", [](ExperimentConfig& c) -> int& { return c.initial.kmeans.max_iterations; });
        dbl("initial.kmeans_convergence_epsilon", [](ExperimentConfig& c) -> double& { return c.initial.kmeans.convergence_epsilon; });

        usz("vns.iter_max", [](ExperimentConfig& c) -> std::size_t& { return c.vns.iter_max; });
        dbl("vns.t0_fraction", [](ExperimentConfig& c) -> double& { return c.vns.t0_fraction; });
        dbl("vns.t0_absolute", [](ExperimentConfig& c) -> double& { return c.vns.t0_absolute; });
        dbl("vns.cooling_rate", [](ExperimentConfig& c) -> double& { return c.vns.cooling_rate; });
        dbl("vns.t_min", [](ExperimentConfig& c) -> double& { return c.vns.t_min; });
        usz("vns.max_no_improve", [](ExperimentConfig& c) -> std::size_t& { return c.vns.max_no_improve; });
        dbl("vns.max_seconds", [](ExperimentConfig& c) -> double& { return c.vns.max_seconds; });
        usz("vns.max_shake_attempts", [](ExperimentConfig& c) -> std::size_t& { return c.vns.max_shake_attempts; });
        usz("vns.max_local_steps", [](ExperimentConfig& c) -> std::size_t& { return c.vns.max_local_steps; });
        dbl("vns.min_improvement", [](ExperimentConfig& c) -> double& { return c.vns.min_improvement; });
        f.push_back({"vns.reference",
                     [](const ExperimentConfig& c) { return std::string(c.vns.reference == AcceptanceReference::Incumbent ? "incumbent" : "shaken-state"); },
                     [](ExperimentConfig& c, const std::string& v) -> std::string {
                         if (v == "incumbent") c.vns.reference = AcceptanceReference::Incumbent;
                         else if (v == "shaken-state") c.vns.reference = AcceptanceReference::ShakenState;
                         else return "'" + v + "' is not one of incumbent, shaken-state";
                         return ""; }});
        f.push_back({"vns.neighbourhoods",
                     [](const ExperimentConfig& c) {
                         if (c.vns.neighbourhoods.empty()) return std::string("default");
                         std::string s;
                         for (std::size_t i = 0; i < c.vns.neighbourhoods.size(); ++i) s += (i ? "," : "") + detail_cfg::op_name(c.vns.neighbourhoods[i]);
                         return s; },
                     [](ExperimentConfig& c, const std::string& v) -> std::string {
                         if (v == "default") { c.vns.neighbourhoods.clear(); return ""; }
                         std::vector<OperatorId> list;
                         std::stringstream ss(v);
                         std::string tok;
                         while (std::getline(ss, tok, ',')) {
                             tok = detail_cfg::trim(tok);
                             bool found = false;
                             for (OperatorId o : {OperatorId::N1, OperatorId::N2, OperatorId::N3, OperatorId::N4, OperatorId::N5, OperatorId::N6, OperatorId::N7, OperatorId::N8})
                                 if (tok == detail_cfg::op_name(o)) { list.push_back(o); found = true; }
                             if (!found) return "'" + tok + "' is not an operator name (N1..N8)";
                         }
                         if (list.empty()) return "empty neighbourhood list (use 'default' for the standard list)";
                         c.vns.neighbourhoods = list;
                         return ""; }});

        i32("shake.max_segment_length", [](ExperimentConfig& c) -> int& { return c.vns.shake.max_segment_length; });
        i32("shake.max_shift", [](ExperimentConfig& c) -> int& { return c.vns.shake.max_shift; });

        f.push_back({"n4.mode",
                     [](const ExperimentConfig& c) { switch (c.vns.n4.mode) { case N4Mode::Relocate: return std::string("relocate"); case N4Mode::Exchange: return std::string("exchange"); case N4Mode::RelocateAndExchange: return std::string("relocate-and-exchange"); } return std::string("?"); },
                     [](ExperimentConfig& c, const std::string& v) -> std::string {
                         if (v == "relocate") c.vns.n4.mode = N4Mode::Relocate;
                         else if (v == "exchange") c.vns.n4.mode = N4Mode::Exchange;
                         else if (v == "relocate-and-exchange") c.vns.n4.mode = N4Mode::RelocateAndExchange;
                         else return "'" + v + "' is not one of relocate, exchange, relocate-and-exchange";
                         return ""; }});
        f.push_back({"n4.affected_trips",
                     [](const ExperimentConfig& c) { return std::string(c.vns.n4.affected_trips == AffectedTripPolicy::Reanchor ? "reanchor" : "reject"); },
                     [](ExperimentConfig& c, const std::string& v) -> std::string {
                         if (v == "reanchor") c.vns.n4.affected_trips = AffectedTripPolicy::Reanchor;
                         else if (v == "reject") c.vns.n4.affected_trips = AffectedTripPolicy::Reject;
                         else return "'" + v + "' is not one of reanchor, reject";
                         return ""; }});
        usz("n4.max_reanchor_variants", [](ExperimentConfig& c) -> std::size_t& { return c.vns.n4.max_reanchor_variants; });
        return f;
    }();
    return F;
}

inline const ConfigField* find_field(const std::string& key) {
    for (const auto& f : config_fields())
        if (f.key == key) return &f;
    return nullptr;
}

// Sets one `key = value`. Returns "" or the problem (unknown key / malformed value).
inline std::string set_config_value(ExperimentConfig& c, const std::string& key, const std::string& value) {
    const ConfigField* f = find_field(key);
    if (!f) return "unknown key '" + key + "'";
    const std::string e = f->set(c, value);
    return e.empty() ? "" : key + ": " + e;
}

struct ConfigParseResult {
    bool ok = true;
    std::vector<std::string> errors;
};

// Parses text into `c` (starting from whatever `c` holds, normally the defaults). Strict: unknown keys, malformed values and
// duplicate keys are errors; every problem is reported with its line number; processing continues so all problems show at once.
inline ConfigParseResult parse_config_text(std::string text, ExperimentConfig& c) {
    ConfigParseResult r;
    if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xEF && static_cast<unsigned char>(text[1]) == 0xBB && static_cast<unsigned char>(text[2]) == 0xBF)
        text.erase(0, 3); // UTF-8 byte-order mark (Windows editors add it)
    std::vector<std::string> seen;
    std::string section;
    std::stringstream in(text);
    std::string line;
    int ln = 0;
    while (std::getline(in, line)) {
        ++ln;
        const std::size_t h = line.find_first_of("#;");
        if (h != std::string::npos) line.erase(h);
        line = detail_cfg::trim(line);
        if (line.empty()) continue;
        auto err = [&](const std::string& m) { r.ok = false; r.errors.push_back("line " + std::to_string(ln) + ": " + m); };
        if (line.front() == '[') {
            if (line.back() != ']' || line.size() < 3) { err("malformed section header '" + line + "'"); continue; }
            section = detail_cfg::trim(line.substr(1, line.size() - 2));
            continue;
        }
        const std::size_t eq = line.find('=');
        if (eq == std::string::npos) { err("expected 'key = value', got '" + line + "'"); continue; }
        std::string key = detail_cfg::trim(line.substr(0, eq)), val = detail_cfg::trim(line.substr(eq + 1));
        if (key.empty()) { err("empty key"); continue; }
        if (!section.empty() && key.find('.') == std::string::npos) key = section + "." + key;
        bool dup = false;
        for (const auto& s : seen) dup = dup || s == key;
        if (dup) { err("duplicate key '" + key + "'"); continue; }
        seen.push_back(key);
        const std::string e = set_config_value(c, key, val);
        if (!e.empty()) err(e);
    }
    return r;
}

inline ConfigParseResult load_config_file(const std::string& path, ExperimentConfig& c) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { ConfigParseResult r; r.ok = false; r.errors.push_back("cannot open configuration file '" + path + "'"); return r; }
    std::stringstream ss;
    ss << f.rdbuf();
    return parse_config_text(ss.str(), c);
}

// Canonical, complete, deterministic text: every key, in table order, doubles printed round-trip exact (%.17g).
inline std::string config_to_text(const ExperimentConfig& c) {
    std::string s = "# resolved configuration (all keys, canonical order); parse it back to reproduce the experiment\n";
    for (const auto& f : config_fields()) s += f.key + " = " + f.get(c) + "\n";
    return s;
}

// The VNS options the runner actually passes (single wiring point; the runner adds only base_seed and run_index per run).
inline tdr::search::VnsOptions effective_vns_options(const ExperimentConfig& c) {
    tdr::search::VnsOptions o = c.vns;
    o.base_seed = c.base.seed.base_seed;
    o.n1_repeats = c.base.neighbourhood.n1_repeats;
    o.n2_repeats = c.base.neighbourhood.n2_repeats;
    o.record_trace = c.output.record_trace;
    return o;
}

// Every problem found (empty = valid). Never throws.
inline std::vector<std::string> validate(const ExperimentConfig& c) {
    std::vector<std::string> e = tdr::config::validate(c.base);
    for (auto& m : tdr::search::validate(effective_vns_options(c))) e.push_back("vns: " + m);
    if (c.experiment.num_datasets < 1) e.push_back("experiment.num_datasets must be >= 1");
    if (c.experiment.runs < 1) e.push_back("experiment.runs must be >= 1");
    if (c.experiment.name.empty()) e.push_back("experiment.name must not be empty");
    if (c.initial.tsp_max_exact_n < 1) e.push_back("initial.tsp_max_exact_n must be >= 1");
    if (c.initial.tsp_heuristic_restarts < 1) e.push_back("initial.tsp_heuristic_restarts must be >= 1");
    if (c.initial.kmeans.max_iterations < 1) e.push_back("initial.kmeans_max_iterations must be >= 1");
    if (!(c.initial.kmeans.convergence_epsilon >= 0.0)) e.push_back("initial.kmeans_convergence_epsilon must be >= 0");
    if (c.vns.shake.max_segment_length < 1) e.push_back("shake.max_segment_length must be >= 1");
    if (c.vns.shake.max_shift < 1) e.push_back("shake.max_shift must be >= 1");
    // dataset indices of the whole experiment must fit
    if (c.base.instance.dataset_index >= 0 && c.experiment.num_datasets >= 1 &&
        static_cast<long long>(c.base.instance.dataset_index) + c.experiment.num_datasets > 2147483647LL)
        e.push_back("instance.dataset_index + experiment.num_datasets overflows");
    return e;
}

} // namespace tdr::config
