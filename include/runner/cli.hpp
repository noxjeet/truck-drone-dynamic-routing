#pragma once

// Command-line front end for the STATIC reproduction (M8, 9 Nov 2026). Thin: parses arguments, builds the configuration (defaults ->
// --config file -> --set overrides, in that order), delegates to the runner and the writer. `cli_main` takes its streams so tests can
// drive it without a process.
//
//   tdr_static run          --out DIR [--config FILE] [--set key=value]... [--overwrite] [--quiet]   (default command)
//   tdr_static check        [--config FILE] [--set key=value]...         validate and print the resolved configuration
//   tdr_static print-config [--config FILE] [--set key=value]...         print the resolved configuration (canonical, complete)
//   tdr_static list-keys                                                  every configuration key with its default
//   tdr_static help | version
//
// Exit codes: 0 success; 1 the experiment ran but reported errors (e.g. infeasible initial solution, failed verification);
// 2 usage or configuration error; 4 output could not be written.

#include <iostream>
#include <string>
#include <vector>

#include "config/experiment_config.hpp"
#include "runner/result_writer.hpp"
#include "runner/static_runner.hpp"

namespace tdr::runner {

struct CliOptions {
    std::string command = "run";
    std::string out_dir;
    std::string config_file;
    std::vector<std::string> overrides; // "key=value"
    bool overwrite = false;
    bool quiet = false;
    std::vector<std::string> errors;
};

inline CliOptions parse_cli(const std::vector<std::string>& args) {
    CliOptions o;
    std::size_t i = 0;
    if (!args.empty() && !args[0].empty() && args[0][0] != '-') { o.command = args[0]; i = 1; }
    auto need = [&](const std::string& flag, std::string& dst) {
        if (i + 1 >= args.size()) { o.errors.push_back(flag + " needs a value"); return false; }
        dst = args[++i];
        return true;
    };
    for (; i < args.size(); ++i) {
        const std::string& a = args[i];
        std::string v;
        if (a == "--out") { if (need(a, v)) o.out_dir = v; }
        else if (a == "--config") { if (need(a, v)) o.config_file = v; }
        else if (a == "--set") { if (need(a, v)) o.overrides.push_back(v); }
        else if (a == "--overwrite") o.overwrite = true;
        else if (a == "--quiet") o.quiet = true;
        else if (a == "--help" || a == "-h") o.command = "help";
        else if (a == "--version") o.command = "version";
        else o.errors.push_back("unknown argument '" + a + "'");
    }
    static const char* known[] = {"run", "check", "print-config", "list-keys", "help", "version"};
    bool ok = false;
    for (const char* k : known) ok = ok || o.command == k;
    if (!ok) o.errors.push_back("unknown command '" + o.command + "'");
    return o;
}

inline std::string usage_text() {
    return std::string(kToolVersion) + "\n"
           "usage:\n"
           "  tdr_static run          --out DIR [--config FILE] [--set key=value]... [--overwrite] [--quiet]\n"
           "  tdr_static check        [--config FILE] [--set key=value]...\n"
           "  tdr_static print-config [--config FILE] [--set key=value]...\n"
           "  tdr_static list-keys\n"
           "  tdr_static help | version\n"
           "configuration precedence: built-in defaults < --config file < --set overrides (later --set wins).\n"
           "exit codes: 0 ok, 1 experiment reported errors, 2 usage/configuration error, 4 output error.\n";
}

// Builds the configuration; returns the problems (empty = ok).
inline std::vector<std::string> build_config(const CliOptions& o, tdr::config::ExperimentConfig& cfg) {
    std::vector<std::string> errs;
    if (!o.config_file.empty()) {
        const auto r = tdr::config::load_config_file(o.config_file, cfg);
        for (const auto& e : r.errors) errs.push_back(o.config_file + ": " + e);
    }
    for (const auto& s : o.overrides) {
        const std::size_t eq = s.find('=');
        if (eq == std::string::npos) { errs.push_back("--set expects key=value, got '" + s + "'"); continue; }
        const std::string e = tdr::config::set_config_value(cfg, tdr::config::detail_cfg::trim(s.substr(0, eq)), tdr::config::detail_cfg::trim(s.substr(eq + 1)));
        if (!e.empty()) errs.push_back("--set " + s + ": " + e);
    }
    if (errs.empty()) for (const auto& e : tdr::config::validate(cfg)) errs.push_back(e);
    return errs;
}

inline int cli_main(const std::vector<std::string>& args, std::ostream& out, std::ostream& err) {
    const CliOptions o = parse_cli(args);
    if (!o.errors.empty()) {
        for (const auto& e : o.errors) err << "error: " << e << "\n";
        err << usage_text();
        return 2;
    }
    if (o.command == "help") { out << usage_text(); return 0; }
    if (o.command == "version") { out << kToolVersion << "\n"; return 0; }
    if (o.command == "list-keys") {
        const tdr::config::ExperimentConfig def;
        for (const auto& f : tdr::config::config_fields()) out << f.key << " = " << f.get(def) << "\n";
        return 0;
    }
    tdr::config::ExperimentConfig cfg;
    const auto cerrs = build_config(o, cfg);
    if (!cerrs.empty()) {
        for (const auto& e : cerrs) err << "error: " << e << "\n";
        return 2;
    }
    if (o.command == "print-config") { out << tdr::config::config_to_text(cfg); return 0; }
    if (o.command == "check") { out << "configuration is valid\n" << tdr::config::config_to_text(cfg); return 0; }

    // run
    if (o.out_dir.empty()) { err << "error: 'run' needs --out DIR\n"; return 2; }
    ProgressFn progress;
    if (!o.quiet) progress = [&](const std::string& m) { out << m << "\n"; };
    const ExperimentResult X = run_experiment(cfg, progress);
    if (X.datasets.empty() && !X.errors.empty()) {
        for (const auto& e : X.errors) err << "error: " << e << "\n";
        return 2;
    }
    const WriteResult W = write_experiment(o.out_dir, X, o.overwrite);
    if (!W.ok) { err << "error: " << W.error << "\n"; return 4; }
    if (!o.quiet) out << "wrote " << W.files.size() << " files to " << o.out_dir << "\n";
    for (const auto& e : X.errors) err << "error: " << e << "\n";
    return X.ok ? 0 : 1;
}

} // namespace tdr::runner
