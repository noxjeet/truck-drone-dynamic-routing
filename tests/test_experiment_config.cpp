// M8: configuration wiring (include/config/experiment_config.hpp). Defaults come from the algorithm structs themselves (no duplicated
// constants); the text format is strict, complete and round-trips exactly; invalid input is reported, never accepted silently.
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#include "config/experiment_config.hpp"

using namespace tdr::config;
using tdr::search::AcceptanceReference;
using tdr::search::AffectedTripPolicy;
using tdr::search::N4Mode;
using tdr::search::OperatorId;

namespace {

int g_failures = 0;
int g_passes = 0;

void check(bool condition, const std::string& description) {
    if (condition) { ++g_passes; std::cout << "[PASS] " << description << "\n"; }
    else { ++g_failures; std::cout << "[FAIL] " << description << "\n"; }
}

std::string set(ExperimentConfig& c, const std::string& k, const std::string& v) { return set_config_value(c, k, v); }

bool has(const std::vector<std::string>& v, const std::string& needle) {
    for (const auto& s : v) if (s.find(needle) != std::string::npos) return true;
    return false;
}

} // namespace

int main() {
    // ------------------------------------------------------------------ defaults come from the algorithm structs (single source)
    {
        const ExperimentConfig c;
        const tdr::search::VnsOptions v;
        const StaticConfig s;
        const tdr::init::KMeansPolicy km;
        check(validate(c).empty(), "the default configuration is valid");
        auto get = [&](const char* k) { return find_field(k)->get(c); };
        check(get("vns.iter_max") == std::to_string(v.iter_max) && get("vns.t0_fraction") == detail_cfg::fmt_double(v.t0_fraction) && get("vns.cooling_rate") == detail_cfg::fmt_double(v.cooling_rate) &&
              get("vns.t_min") == detail_cfg::fmt_double(v.t_min) && get("vns.max_shake_attempts") == std::to_string(v.max_shake_attempts) && get("vns.max_local_steps") == std::to_string(v.max_local_steps) &&
              get("vns.min_improvement") == "0" && get("vns.max_no_improve") == "0" && get("vns.max_seconds") == "0" && get("vns.reference") == "incumbent" && get("vns.neighbourhoods") == "default",
              "VNS defaults equal VnsOptions{} (pilot values; threshold 0 = strict '<'; step guard 1000)");
        check(get("shake.max_segment_length") == std::to_string(v.shake.max_segment_length) && get("shake.max_shift") == std::to_string(v.shake.max_shift) &&
              get("n4.max_reanchor_variants") == std::to_string(v.n4.max_reanchor_variants) && get("n4.affected_trips") == "reanchor" && get("n4.mode") == "relocate-and-exchange",
              "shake and N4 defaults equal ShakeOptions{} / N4Options{}");
        check(get("instance.num_customers") == std::to_string(s.instance.num_customers) && get("vehicle.endurance_s") == detail_cfg::fmt_double(s.vehicle.endurance_s) &&
              get("flp.unit_division_length_m") == detail_cfg::fmt_double(s.flp.unit_division_length_m) && get("neighbourhood.adjacency_window") == std::to_string(s.neighbourhood.adjacency_window) &&
              get("neighbourhood.n1_repeats") == "2" && get("neighbourhood.n2_repeats") == "2" && get("seed.base_seed") == std::to_string(s.seed.base_seed),
              "instance, vehicle, FLP, neighbourhood and seed defaults equal StaticConfig{}");
        check(get("initial.kmeans_max_iterations") == std::to_string(km.max_iterations) && get("initial.kmeans_convergence_epsilon") == detail_cfg::fmt_double(km.convergence_epsilon) &&
              get("initial.tsp_max_exact_n") == "21" && get("initial.tsp_heuristic_restarts") == "8" && get("initial.tsp_heuristic_seed") == "42",
              "F1/F2 constants default to the values that were hard-coded before M8 (TSP 21 / 8 / 42, KMeansPolicy{})");
        check(get("experiment.runs") == "10" && get("experiment.num_datasets") == "1" && get("experiment.first_run_index") == "0" && get("output.record_trace") == "false" && get("output.write_solutions") == "true",
              "experiment defaults: 10 runs, 1 dataset, run indices from 0, no trace, solutions written");
        std::set<std::string> keys;
        bool unique = true;
        for (const auto& f : config_fields()) unique = unique && keys.insert(f.key).second;
        check(unique && config_fields().size() == 48, "every key appears once in the field table (" + std::to_string(config_fields().size()) + " keys)");
    }

    // ------------------------------------------------------------------ every key accepts its own printed value
    {
        ExperimentConfig c;
        bool all = true;
        for (const auto& f : config_fields()) {
            ExperimentConfig d;
            if (!f.set(d, f.get(c)).empty()) { all = false; std::cout << "  key rejects its own default: " << f.key << "\n"; }
        }
        check(all, "each field's setter accepts the value its getter prints");
    }

    // ------------------------------------------------------------------ round trip with every kind of value non-default
    {
        ExperimentConfig c;
        c.base.instance.num_customers = 37; c.base.instance.area_side_m = 12345.678901234567; c.base.instance.depot_placement = DepotPlacement::Explicit;
        c.base.instance.depot_x_m = 1.0 / 3.0; c.base.instance.depot_y_m = 1e-9; c.base.instance.dataset_index = 3;
        c.base.vehicle.endurance_s = 0.1 + 0.2; c.base.vehicle.num_drones = 5; c.base.flp.unit_division_length_m = 1234.5;
        c.base.neighbourhood.adjacency_window = 4; c.base.neighbourhood.n1_repeats = 3; c.base.neighbourhood.n2_repeats = 1;
        c.base.seed.base_seed = 18446744073709551615ULL;
        c.initial.tsp_max_exact_n = 7; c.initial.tsp_heuristic_restarts = 3; c.initial.tsp_heuristic_seed = 99; c.initial.kmeans.max_iterations = 17; c.initial.kmeans.convergence_epsilon = 1e-7;
        c.vns.iter_max = 123; c.vns.t0_fraction = 0.0123456789; c.vns.t0_absolute = 5.5; c.vns.cooling_rate = 0.987; c.vns.t_min = 3e-11; c.vns.max_no_improve = 9; c.vns.max_seconds = 12.5;
        c.vns.max_shake_attempts = 4; c.vns.max_local_steps = 77; c.vns.min_improvement = 0.25; c.vns.reference = AcceptanceReference::ShakenState;
        c.vns.neighbourhoods = {OperatorId::N7, OperatorId::N1, OperatorId::N4};
        c.vns.shake.max_segment_length = 5; c.vns.shake.max_shift = 2;
        c.vns.n4.mode = N4Mode::Exchange; c.vns.n4.affected_trips = AffectedTripPolicy::Reject; c.vns.n4.max_reanchor_variants = 0;
        c.experiment.name = "round trip"; c.experiment.num_datasets = 4; c.experiment.runs = 10; c.experiment.first_run_index = 5;
        c.output.record_trace = true; c.output.write_solutions = false;
        const std::string t = config_to_text(c);
        ExperimentConfig d;
        const auto r = parse_config_text(t, d);
        check(r.ok && r.errors.empty(), "the canonical text parses without errors");
        check(config_to_text(d) == t, "dump -> parse -> dump is identical (all value kinds, %.17g doubles, 64-bit seed, enum strings, operator list)");
        check(d.base.vehicle.endurance_s == 0.1 + 0.2 && d.base.instance.depot_x_m == 1.0 / 3.0 && d.base.seed.base_seed == 18446744073709551615ULL && d.vns.neighbourhoods == c.vns.neighbourhoods && d.experiment.name == "round trip",
              "parsed values are bit-identical to the originals");
        bool every = true;
        for (const auto& f : config_fields()) every = every && t.find("\n" + f.key + " = ") != std::string::npos && f.get(d) == f.get(c);
        check(every, "the dump lists every key and every parsed field equals the original (checked field by field through the table)");
    }

    // ------------------------------------------------------------------ parser: layout, sections, comments, strictness
    {
        ExperimentConfig c;
        const std::string text =
            "# comment\n"
            "  seed.base_seed =  777   # trailing comment\n"
            "[vns]\n"
            "iter_max = 55\n"
            "cooling_rate=0.9\n"
            "; another comment\n"
            "[instance]\n"
            "num_customers = 12\n"
            "[shake]\n"
            "max_shift = 4\n";
        const auto r = parse_config_text(text, c);
        check(r.ok && c.base.seed.base_seed == 777 && c.vns.iter_max == 55 && c.vns.cooling_rate == 0.9 && c.base.instance.num_customers == 12 && c.vns.shake.max_shift == 4,
              "section prefixes, dotted keys, whitespace and # / ; comments");
        ExperimentConfig d;
        const auto bad = parse_config_text("vns.iter_max = 5\nvns.iter_max = 6\nbogus.key = 1\nvns.cooling_rate = abc\nthis line has no equals\n[broken\nvns.max_seconds = nan\nvns.max_seconds = inf\n", d);
        check(!bad.ok && has(bad.errors, "line 2: duplicate key") && has(bad.errors, "line 3: unknown key 'bogus.key'") && has(bad.errors, "line 4: vns.cooling_rate") && has(bad.errors, "line 5: expected 'key = value'") && has(bad.errors, "line 6: malformed section"),
              "every problem is reported with its line number (duplicate, unknown key, bad number, missing '=', bad section)");
        check(bad.errors.size() == 7, "all problems are collected in one pass (the NaN value and the duplicate that follows it are both reported) -> " + std::to_string(bad.errors.size()) + " errors");
        ExperimentConfig e;
        check(!set(e, "vns.iter_max", "-3").empty() && !set(e, "vns.iter_max", "12x").empty() && !set(e, "vns.iter_max", "").empty() && !set(e, "vns.iter_max", "1.5").empty() && !set(e, "seed.base_seed", "-1").empty() &&
              !set(e, "seed.base_seed", "99999999999999999999999").empty() && !set(e, "instance.num_customers", "99999999999").empty() && !set(e, "vns.t0_fraction", "1e999").empty() && !set(e, "vns.t0_fraction", "nan").empty() &&
              !set(e, "output.record_trace", "yes").empty() && !set(e, "vns.reference", "best").empty() && !set(e, "n4.mode", "swap").empty() && !set(e, "n4.affected_trips", "repair").empty() &&
              !set(e, "instance.depot_placement", "middle").empty() && !set(e, "vns.neighbourhoods", "N9").empty() && !set(e, "vns.neighbourhoods", "").empty() && !set(e, "experiment.name", "").empty() && !set(e, "no.such", "1").empty() && !set(e, "instance.num_customers", "12x").empty() && !set(e, "instance.num_customers", "1.5").empty() && !set(e, "instance.num_customers", "").empty() && !set(e, "experiment.runs", "3 4").empty(),
              "malformed values are rejected: negative / fractional / empty / overflowing integers, NaN, inf, bad booleans, bad enum names, bad operator lists, unknown keys");
        check(c.vns.iter_max == 55 && e.vns.iter_max == tdr::search::VnsOptions{}.iter_max, "a rejected value leaves the field unchanged");
        ExperimentConfig f;
        check(set(f, "vns.neighbourhoods", "N4, N5 ,N8").empty() && f.vns.neighbourhoods == std::vector<OperatorId>({OperatorId::N4, OperatorId::N5, OperatorId::N8}) && set(f, "vns.neighbourhoods", "default").empty() && f.vns.neighbourhoods.empty(),
              "operator list parsing (spaces tolerated, 'default' restores the standard list)");
        ExperimentConfig g;
        check(set(g, "output.record_trace", "1").empty() && g.output.record_trace && set(g, "output.record_trace", "false").empty() && !g.output.record_trace, "booleans accept true / false / 1 / 0");
    }

    // ------------------------------------------------------------------ Windows-style text: CRLF line ends and a UTF-8 byte-order mark
    {
        ExperimentConfig c;
        const std::string text = "\xEF\xBB\xBF# written by Notepad\r\n[vns]\r\niter_max = 12\r\n\r\n[experiment]\r\nname = win\r\nruns = 2\r\n";
        const auto r = parse_config_text(text, c);
        check(r.ok && c.vns.iter_max == 12 && c.experiment.name == "win" && c.experiment.runs == 2, "CRLF line ends and a leading UTF-8 BOM are accepted (value and name carry no stray carriage return)");
    }

    // ------------------------------------------------------------------ validation of the complete configuration
    {
        auto bad = [&](auto mut, const std::string& needle) { ExperimentConfig c; mut(c); const auto e = validate(c); return has(e, needle); };
        check(bad([](ExperimentConfig& c) { c.experiment.runs = 0; }, "experiment.runs") && bad([](ExperimentConfig& c) { c.experiment.num_datasets = 0; }, "experiment.num_datasets") &&
              bad([](ExperimentConfig& c) { c.experiment.name = ""; }, "experiment.name") && bad([](ExperimentConfig& c) { c.base.instance.num_customers = 0; }, "num_customers") &&
              bad([](ExperimentConfig& c) { c.base.vehicle.endurance_s = -1; }, "endurance") && bad([](ExperimentConfig& c) { c.base.neighbourhood.n1_repeats = 0; }, "n1_repeats") &&
              bad([](ExperimentConfig& c) { c.vns.cooling_rate = 0; }, "cooling_rate") && bad([](ExperimentConfig& c) { c.vns.t_min = 0; }, "t_min") && bad([](ExperimentConfig& c) { c.vns.max_shake_attempts = 0; }, "max_shake_attempts") &&
              bad([](ExperimentConfig& c) { c.vns.min_improvement = -1; }, "min_improvement") && bad([](ExperimentConfig& c) { c.initial.tsp_max_exact_n = 0; }, "tsp_max_exact_n") &&
              bad([](ExperimentConfig& c) { c.initial.tsp_heuristic_restarts = 0; }, "tsp_heuristic_restarts") && bad([](ExperimentConfig& c) { c.initial.kmeans.max_iterations = 0; }, "kmeans_max_iterations") &&
              bad([](ExperimentConfig& c) { c.vns.shake.max_segment_length = 0; }, "shake.max_segment_length") && bad([](ExperimentConfig& c) { c.vns.shake.max_shift = 0; }, "shake.max_shift") &&
              bad([](ExperimentConfig& c) { c.base.instance.dataset_index = 2147483646; c.experiment.num_datasets = 5; }, "overflows"),
              "validate() reports every invalid class of value (experiment, instance, vehicle, neighbourhood, VNS, F1/F2, shake, dataset range)");
        ExperimentConfig ok;
        ok.vns.max_local_steps = 0; ok.vns.iter_max = 0; ok.vns.cooling_rate = 1.0; ok.vns.n4.max_reanchor_variants = 0;
        check(validate(ok).empty(), "documented special values stay valid (unlimited guard, iter_max 0, cooling 1, unlimited re-anchoring)");
    }

    // ------------------------------------------------------------------ one wiring point into the VNS options
    {
        ExperimentConfig c;
        c.base.seed.base_seed = 424242; c.base.neighbourhood.n1_repeats = 3; c.base.neighbourhood.n2_repeats = 1; c.output.record_trace = true;
        c.vns.base_seed = 1; c.vns.n1_repeats = 9; c.vns.n2_repeats = 9; c.vns.record_trace = false; c.vns.iter_max = 31; c.vns.run_index = 5;
        const auto o = effective_vns_options(c);
        check(o.base_seed == 424242 && o.n1_repeats == 3 && o.n2_repeats == 1 && o.record_trace && o.iter_max == 31 && o.run_index == 5,
              "effective VNS options: base seed, N1/N2 repeats and trace come from seed./neighbourhood./output.; every other field passes through untouched");
    }

    // ------------------------------------------------------------------ files
    {
        const auto dir = std::filesystem::temp_directory_path() / "tdr_m8_cfg_test";
        std::filesystem::create_directories(dir);
        const auto good = (dir / "good.cfg").string(), bad = (dir / "bad.cfg").string();
        { std::ofstream(good) << "[vns]\niter_max = 11\n[experiment]\nruns = 2\n"; }
        { std::ofstream(bad) << "vns.iter_max = x\nunknown = 3\n"; }
        ExperimentConfig c, d;
        const auto r1 = load_config_file(good, c), r2 = load_config_file(bad, d), r3 = load_config_file((dir / "missing.cfg").string(), d);
        check(r1.ok && c.vns.iter_max == 11 && c.experiment.runs == 2, "a configuration file is read");
        check(!r2.ok && r2.errors.size() == 2 && !r3.ok && has(r3.errors, "cannot open"), "a bad file reports all its errors; a missing file is an error");
        std::filesystem::remove_all(dir);
    }

    std::cout << "\n" << g_passes << " passed, " << g_failures << " failed\n";
    if (g_failures == 0) std::cout << "ALL TESTS PASSED\n";
    return g_failures == 0 ? 0 : 1;
}
