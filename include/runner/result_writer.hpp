#pragma once

// Result writer for the STATIC reproduction (M8, 9 Nov 2026). Turns an ExperimentResult into machine-readable files; never runs
// the algorithm. Text generation (`*_json`, `*_csv`) is separate from file output (`write_experiment`) so it can be tested
// without a file system.
//
// Files (all UTF-8, "\n" line ends, numbers printed round-trip exact with %.17g, objectives in seconds with minutes beside them (D10)):
//   config.resolved.cfg  complete canonical configuration (parse it back to reproduce the experiment)
//   results.json         instance / initial solution / every run (counters, guard, shakes, operators, best solution) / per-dataset summary
//   runs.csv             one row per (dataset, run): the headline numbers of results.json
//   traces.csv           per-pass trace of every run (only when output.record_trace)
//   timing.json          wall-clock seconds -- the ONLY non-deterministic content
//   manifest.json        file list with sizes and FNV-1a 64 digests of the deterministic files; `deterministic` is false when vns.max_seconds > 0
// All files except timing.json (and manifest.json, which lists timing.json without a digest) are byte-identical across repeated runs
// of the same configuration.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "config/experiment_config.hpp"
#include "runner/static_runner.hpp"

namespace tdr::runner {

inline constexpr const char* kToolVersion = "tdr_static 1.1 (M8)";

inline std::string fnv1a64_hex(const std::string& s) {
    std::uint64_t h = 14695981039346656037ULL;
    for (unsigned char c : s) { h ^= c; h *= 1099511628211ULL; }
    char buf[20];
    std::snprintf(buf, sizeof buf, "%016llx", static_cast<unsigned long long>(h));
    return buf;
}

// ------------------------------------------------------------------------------------------------ minimal JSON writer
class JsonWriter {
public:
    void begin_object() { open('{'); }
    void end_object() { close('}'); }
    void begin_array(bool inline_items = false) { open('['); frames_.back().inline_items = inline_items; }
    void end_array() { close(']'); }
    void key(const std::string& k) { sep(); str_(k); s_ += ": "; pending_value_ = true; }
    void value(const std::string& v) { sep(); str_(v); }
    void value(const char* v) { value(std::string(v)); }
    void value(bool v) { sep(); s_ += v ? "true" : "false"; }
    void value(double v) { sep(); if (!std::isfinite(v)) { s_ += "null"; return; } char b[40]; std::snprintf(b, sizeof b, "%.17g", v); s_ += b; }
    void value(long long v) { sep(); s_ += std::to_string(v); }
    void value(unsigned long long v) { sep(); s_ += std::to_string(v); }
    void value(int v) { value(static_cast<long long>(v)); }
    void value(unsigned v) { value(static_cast<unsigned long long>(v)); }
    void value(long v) { value(static_cast<long long>(v)); }
    void value(unsigned long v) { value(static_cast<unsigned long long>(v)); }
    void null() { sep(); s_ += "null"; }
    template <class V> void field(const std::string& k, const V& v) { key(k); value(v); }
    const std::string& str() const { return s_; }
    std::string finish() { return s_ + "\n"; }

private:
    struct Frame { bool first = true; bool inline_items = false; };
    void open(char c) { sep(); s_ += c; frames_.push_back({}); }
    void close(char c) {
        const bool had = !frames_.back().first;
        const bool inl = frames_.back().inline_items;
        frames_.pop_back();
        if (had && !inl) { s_ += "\n"; indent(); }
        s_ += c;
    }
    void sep() {
        if (pending_value_) { pending_value_ = false; return; }
        if (frames_.empty()) return;
        Frame& f = frames_.back();
        if (!f.first) s_ += f.inline_items ? ", " : ",";
        if (!f.inline_items) { s_ += "\n"; indent(); }
        f.first = false;
    }
    void indent() { s_.append(frames_.size() * 2, ' '); }
    void str_(const std::string& v) {
        s_ += '"';
        for (unsigned char c : v) {
            switch (c) {
                case '"': s_ += "\\\""; break;
                case '\\': s_ += "\\\\"; break;
                case '\n': s_ += "\\n"; break;
                case '\r': s_ += "\\r"; break;
                case '\t': s_ += "\\t"; break;
                default:
                    if (c < 0x20) { char b[8]; std::snprintf(b, sizeof b, "\\u%04x", c); s_ += b; }
                    else s_ += static_cast<char>(c);
            }
        }
        s_ += '"';
    }
    std::string s_;
    std::vector<Frame> frames_;
    bool pending_value_ = false;
};

namespace detail_writer {

inline const char* status_name(tdr::search::VnsStatus s) {
    switch (s) {
        case tdr::search::VnsStatus::Ok: return "ok";
        case tdr::search::VnsStatus::InvalidOptions: return "invalid-options";
        case tdr::search::VnsStatus::InitialInfeasible: return "initial-infeasible";
    }
    return "?";
}

inline const char* op_label(std::size_t i) {
    static const char* n[8] = {"N1", "N2", "N3", "N4", "N5", "N6", "N7", "N8"};
    return i < 8 ? n[i] : "?";
}

inline void write_solution(JsonWriter& j, const tdr::model::Solution& s) {
    j.begin_object();
    j.key("truck_route");
    j.begin_array(true);
    for (auto id : s.truck_route) j.value(static_cast<long long>(id));
    j.end_array();
    j.key("drone_trips");
    j.begin_array();
    for (const auto& t : s.drone_trips) {
        j.begin_object();
        j.field("drone", t.drone_id);
        j.field("launch", static_cast<long long>(t.launch_node));
        j.field("customer", static_cast<long long>(t.service_customer));
        j.field("recovery", static_cast<long long>(t.recovery_node));
        j.end_object();
    }
    j.end_array();
    j.end_object();
}

inline void write_objective(JsonWriter& j, const char* base, double seconds) {
    j.field(std::string(base) + "_s", seconds);
    j.field(std::string(base) + "_min", seconds / 60.0);
}

inline double improvement_pct(const tdr::search::VnsResult& v) {
    return v.initial_objective > 0.0 ? 100.0 * (v.initial_objective - v.best_objective) / v.initial_objective : 0.0;
}

} // namespace detail_writer

inline std::string results_json(const ExperimentResult& X) {
    using namespace detail_writer;
    const auto& cfg = X.config;
    JsonWriter j;
    j.begin_object();
    j.field("schema", "tdr.static.results/2");
    j.field("tool", kToolVersion);
    j.key("units");
    j.begin_object();
    j.field("time", "seconds");
    j.field("coordinates", "metres");
    j.field("objective", "sum over all customers of the service arrival time (seconds), lower is better; *_min fields are the same value in minutes");
    j.end_object();
    j.key("experiment");
    j.begin_object();
    j.field("name", cfg.experiment.name);
    j.field("runs_per_dataset", cfg.experiment.runs);
    j.field("num_datasets", cfg.experiment.num_datasets);
    j.field("first_dataset_index", cfg.base.instance.dataset_index);
    j.field("first_run_index", static_cast<unsigned long long>(cfg.experiment.first_run_index));
    j.field("base_seed", static_cast<unsigned long long>(cfg.base.seed.base_seed));
    j.field("config_digest", fnv1a64_hex(tdr::config::config_to_text(cfg)));
    j.field("deterministic", !(cfg.vns.max_seconds > 0.0));
    j.field("ok", X.ok);
    j.key("errors");
    j.begin_array();
    for (const auto& e : X.errors) j.value(e);
    j.end_array();
    j.end_object();

    j.key("datasets");
    j.begin_array();
    for (const auto& D : X.datasets) {
        j.begin_object();
        j.field("dataset_index", D.dataset_index);
        j.field("instance_seed", static_cast<unsigned long long>(D.instance_seed));
        j.field("num_customers", D.num_customers);
        j.field("num_flp_candidates", static_cast<unsigned long long>(D.num_flp_candidates));
        j.key("initial");
        j.begin_object();
        j.field("feasible", D.initial_feasible);
        write_objective(j, "objective", D.initial_objective);
        j.field("num_trips", static_cast<unsigned long long>(D.initial.drone_trips.size()));
        j.key("diagnostics");
        j.begin_array();
        for (const auto& m : D.diagnostics) j.value(m);
        j.end_array();
        if (cfg.output.write_solutions) { j.key("solution"); write_solution(j, D.initial); }
        j.end_object();

        j.key("nodes");
        j.begin_array();
        for (const auto& n : D.nodes) {
            j.begin_object();
            j.field("id", static_cast<long long>(n.id));
            j.field("type", n.type);
            j.field("x_m", n.x);
            j.field("y_m", n.y);
            if (n.type == "flp") {
                j.field("arc_from", static_cast<long long>(n.arc_from));
                j.field("arc_to", static_cast<long long>(n.arc_to));
                j.field("position_ratio", n.position_ratio);
            }
            j.end_object();
        }
        j.end_array();

        j.key("runs");
        j.begin_array();
        double mn = 0, mx = 0, sum = 0, sumpct = 0;
        std::size_t cnt = 0, best_run = 0;
        for (const auto& R : D.runs) {
            const auto& v = R.vns;
            j.begin_object();
            j.field("run_index", static_cast<unsigned long long>(R.run_index));
            j.field("status", status_name(v.status));
            j.field("stop", tdr::search::vns_stop_name(v.stop));
            j.field("verified", R.verified);
            if (!R.verify_note.empty()) j.field("verify_note", R.verify_note);
            write_objective(j, "initial_objective", v.initial_objective);
            write_objective(j, "best_objective", v.best_objective);
            write_objective(j, "final_current_objective", v.current_objective);
            j.field("improvement_pct", improvement_pct(v));
            j.field("iterations", static_cast<unsigned long long>(v.iterations));
            j.field("t0", v.t0);
            j.field("final_temperature", v.final_temperature);
            j.field("best_updates", static_cast<unsigned long long>(v.best_updates));
            j.field("last_best_iteration", static_cast<unsigned long long>(v.last_best_iteration));
            j.field("improving_moves", static_cast<unsigned long long>(v.improving_moves));
            j.field("sa_accepted_moves", static_cast<unsigned long long>(v.sa_accepted_moves));
            j.key("passes");
            j.begin_object();
            j.field("completed", static_cast<unsigned long long>(v.passes_completed));
            j.field("skipped", static_cast<unsigned long long>(v.iterations_skipped));
            j.field("guard_truncated", static_cast<unsigned long long>(v.passes_truncated));
            j.field("guard_triggered", v.guard_triggered());
            j.field("local_steps_total", static_cast<unsigned long long>(v.local_steps_total));
            j.field("max_local_steps_in_pass", static_cast<unsigned long long>(v.max_local_steps_in_pass));
            j.end_object();
            j.key("shakes");
            j.begin_object();
            j.field("applied", static_cast<unsigned long long>(v.shakes_applied));
            j.field("not_applicable_redraws", static_cast<unsigned long long>(v.shakes_not_applicable));
            j.field("infeasible_redraws", static_cast<unsigned long long>(v.shakes_infeasible));
            j.key("applied_by_kind");
            j.begin_object();
            static const char* kinds[4] = {"reverse", "exchange", "shift", "drone_remove"};
            for (std::size_t k = 0; k < 4; ++k) j.field(kinds[k], static_cast<unsigned long long>(v.shakes_applied_by_kind[k]));
            j.end_object();
            j.end_object();
            j.key("operators");
            j.begin_array();
            for (std::size_t k = 0; k < 8; ++k) {
                const auto& o = v.ops[k];
                j.begin_object();
                j.field("op", op_label(k));
                j.field("calls", static_cast<unsigned long long>(o.calls));
                j.field("no_anchor", static_cast<unsigned long long>(o.no_anchor));
                j.field("no_candidate", static_cast<unsigned long long>(o.no_candidate));
                j.field("improving", static_cast<unsigned long long>(o.improving));
                j.field("sa_accepted", static_cast<unsigned long long>(o.sa_accepted));
                j.field("sa_rejected", static_cast<unsigned long long>(o.sa_rejected));
                j.end_object();
            }
            j.end_array();
            if (v.status == tdr::search::VnsStatus::Ok) {
                j.field("num_trips", static_cast<unsigned long long>(v.best.drone_trips.size()));
                if (cfg.output.write_solutions) { j.key("best_solution"); write_solution(j, v.best); }
                if (cnt == 0 || v.best_objective < mn) { mn = v.best_objective; best_run = static_cast<std::size_t>(R.run_index); }
                if (cnt == 0 || v.best_objective > mx) mx = v.best_objective;
                sum += v.best_objective;
                sumpct += improvement_pct(v);
                ++cnt;
            }
            j.end_object();
        }
        j.end_array();

        j.key("summary");
        j.begin_object();
        j.field("runs_ok", static_cast<unsigned long long>(cnt));
        if (cnt > 0) {
            const double mean = sum / static_cast<double>(cnt);
            double ss = 0.0;
            for (const auto& R : D.runs) if (R.vns.status == tdr::search::VnsStatus::Ok) ss += (R.vns.best_objective - mean) * (R.vns.best_objective - mean);
            write_objective(j, "best_objective", mn);
            j.field("best_run_index", static_cast<unsigned long long>(best_run));
            write_objective(j, "mean_best_objective", mean);
            write_objective(j, "worst_best_objective", mx);
            j.field("std_best_objective_s", cnt > 1 ? std::sqrt(ss / static_cast<double>(cnt - 1)) : 0.0);
            j.field("mean_improvement_pct", sumpct / static_cast<double>(cnt));
        }
        j.end_object();
        j.end_object();
    }
    j.end_array();
    j.end_object();
    return j.finish();
}

inline std::string runs_csv(const ExperimentResult& X) {
    using namespace detail_writer;
    char b[48];
    auto d = [&](double v) { std::snprintf(b, sizeof b, "%.17g", v); return std::string(b); };
    std::string s = "dataset_index,run_index,status,stop,verified,initial_objective_s,best_objective_s,best_objective_min,improvement_pct,iterations,passes_completed,passes_skipped,passes_guard_truncated,guard_triggered,shakes_applied,shakes_not_applicable,shakes_infeasible,improving_moves,sa_accepted_moves,best_updates,last_best_iteration,t0,final_temperature,num_trips\n";
    for (const auto& D : X.datasets)
        for (const auto& R : D.runs) {
            const auto& v = R.vns;
            s += std::to_string(D.dataset_index) + "," + std::to_string(R.run_index) + "," + status_name(v.status) + "," + tdr::search::vns_stop_name(v.stop) + "," + (R.verified ? "true" : "false") + "," +
                 d(v.initial_objective) + "," + d(v.best_objective) + "," + d(v.best_objective / 60.0) + "," + d(improvement_pct(v)) + "," + std::to_string(v.iterations) + "," + std::to_string(v.passes_completed) + "," +
                 std::to_string(v.iterations_skipped) + "," + std::to_string(v.passes_truncated) + "," + (v.guard_triggered() ? "true" : "false") + "," + std::to_string(v.shakes_applied) + "," +
                 std::to_string(v.shakes_not_applicable) + "," + std::to_string(v.shakes_infeasible) + "," + std::to_string(v.improving_moves) + "," + std::to_string(v.sa_accepted_moves) + "," +
                 std::to_string(v.best_updates) + "," + std::to_string(v.last_best_iteration) + "," + d(v.t0) + "," + d(v.final_temperature) + "," + std::to_string(v.best.drone_trips.size()) + "\n";
        }
    return s;
}

inline std::string traces_csv(const ExperimentResult& X) {
    char b[48];
    auto d = [&](double v) { std::snprintf(b, sizeof b, "%.17g", v); return std::string(b); };
    std::string s = "dataset_index,run_index,iter,temperature,shaken,shake_kind,pass_end,local_steps,moves,current_objective_s,best_objective_s\n";
    for (const auto& D : X.datasets)
        for (const auto& R : D.runs)
            for (const auto& t : R.vns.trace)
                s += std::to_string(D.dataset_index) + "," + std::to_string(R.run_index) + "," + std::to_string(t.iter) + "," + d(t.temperature) + "," + (t.shaken ? "true" : "false") + "," +
                     std::to_string(t.shake_kind) + "," + tdr::search::vns_pass_end_name(t.end) + "," + std::to_string(t.local_steps) + "," + std::to_string(t.moves) + "," + d(t.current) + "," + d(t.best) + "\n";
    return s;
}

// Wall clock only. The one non-deterministic file.
inline std::string timing_json(const ExperimentResult& X) {
    JsonWriter j;
    j.begin_object();
    j.field("schema", "tdr.static.timing/1");
    j.field("note", "wall-clock seconds; NOT reproducible, excluded from the manifest digests");
    j.field("total_seconds", X.total_seconds);
    j.key("runs");
    j.begin_array();
    for (const auto& D : X.datasets)
        for (const auto& R : D.runs) {
            j.begin_object();
            j.field("dataset_index", D.dataset_index);
            j.field("run_index", static_cast<unsigned long long>(R.run_index));
            j.field("seconds", R.vns.seconds);
            j.end_object();
        }
    j.end_array();
    j.end_object();
    return j.finish();
}

struct OutputFile {
    std::string name;
    std::string content;
    bool deterministic = true;
};

inline std::vector<OutputFile> render_outputs(const ExperimentResult& X) {
    std::vector<OutputFile> v;
    v.push_back({"config.resolved.cfg", tdr::config::config_to_text(X.config), true});
    v.push_back({"results.json", results_json(X), true});
    v.push_back({"runs.csv", runs_csv(X), true});
    if (X.config.output.record_trace) v.push_back({"traces.csv", traces_csv(X), true});
    v.push_back({"timing.json", timing_json(X), false});
    return v;
}

inline std::string manifest_json(const ExperimentResult& X, const std::vector<OutputFile>& files) {
    JsonWriter j;
    j.begin_object();
    j.field("schema", "tdr.static.manifest/1");
    j.field("tool", kToolVersion);
    j.field("experiment", X.config.experiment.name);
    j.field("deterministic", !(X.config.vns.max_seconds > 0.0));
    j.field("ok", X.ok);
    j.key("files");
    j.begin_array();
    for (const auto& f : files) {
        j.begin_object();
        j.field("name", f.name);
        j.field("bytes", static_cast<unsigned long long>(f.content.size()));
        if (f.deterministic) j.field("fnv1a64", fnv1a64_hex(f.content)); else j.field("fnv1a64", "not-reproducible");
        j.end_object();
    }
    j.end_array();
    j.end_object();
    return j.finish();
}

struct WriteResult {
    bool ok = true;
    std::string error;
    std::vector<std::string> files;
};

// Writes every file via "<name>.tmp" then rename. Refuses to touch a directory that already holds a results.json unless `overwrite`.
inline WriteResult write_experiment(const std::string& dir, const ExperimentResult& X, bool overwrite) {
    namespace fs = std::filesystem;
    WriteResult W;
    std::error_code ec;
    const fs::path root(dir);
    if (fs::exists(root / "results.json", ec) && !overwrite) {
        W.ok = false;
        W.error = "'" + (root / "results.json").string() + "' already exists (use --overwrite to replace it)";
        return W;
    }
    fs::create_directories(root, ec);
    if (ec) { W.ok = false; W.error = "cannot create directory '" + dir + "': " + ec.message(); return W; }
    std::vector<OutputFile> files = render_outputs(X);
    files.push_back({"manifest.json", manifest_json(X, files), true});
    for (const auto& f : files) {
        const fs::path tmp = root / (f.name + ".tmp"), dst = root / f.name;
        {
            std::ofstream o(tmp, std::ios::binary | std::ios::trunc);
            if (!o) { W.ok = false; W.error = "cannot write '" + tmp.string() + "'"; return W; }
            o.write(f.content.data(), static_cast<std::streamsize>(f.content.size()));
            o.flush();
            if (!o) { W.ok = false; W.error = "write failed for '" + tmp.string() + "'"; return W; }
        }
        fs::rename(tmp, dst, ec);
        if (ec) { fs::remove(dst, ec); ec.clear(); fs::rename(tmp, dst, ec); }
        if (ec) { W.ok = false; W.error = "cannot move '" + tmp.string() + "' to '" + dst.string() + "': " + ec.message(); return W; }
        W.files.push_back(f.name);
    }
    if (!X.config.output.record_trace) {
        // a stale traces.csv from an earlier run in the same directory would contradict the current configuration
        fs::remove(root / "traces.csv", ec);
    }
    return W;
}

} // namespace tdr::runner
