import os, shutil, subprocess, sys
# M8 mutation harness: injects one bug at a time into a COPY of include/ and tests/ and checks that the M8 tests (test_experiment_config, test_runner_cli) fail.
# usage: TDR_REPO=<repo> TDR_LIB=<path of libtdr_core.a from a Release build> TDR_WORK=<scratch dir> python3 m8_mutate.py [name-prefix ...]
# The repository is never modified. 'CW-*' mutants are applied to every occurrence of their pattern.
R = os.environ.get('TDR_REPO', os.getcwd())
W = os.environ.get('TDR_WORK', '/tmp/tdr_m8_mut') + '/w'
LIB = os.environ['TDR_LIB']


CF = 'include/config/experiment_config.hpp'
RH = 'include/runner/static_runner.hpp'
WH = 'include/runner/result_writer.hpp'
CH = 'include/runner/cli.hpp'
M = [
 ('M-01 doubles printed with 6 digits', CF, '"%.17g"', '"%.6g"'),
 ('M-02 duplicate keys accepted', CF, 'if (dup) {', 'if (false) {'),
 ('M-03 section prefix not applied', CF, 'if (!section.empty() && key.find(\'.\') == std::string::npos) key = section + "." + key;', ';'),
 ('M-04 ; comments not stripped', CF, 'line.find_first_of("#;")', 'line.find_first_of("#")'),
 ('M-05 unsigned accepts a sign', CF, "if (v[0] == '-' || v[0] == '+') return", 'if (false) return'),
 ('M-06 non-finite numbers accepted', CF, 'if (!std::isfinite(d)) return "\'" + v + "\' is not finite";', ''),
 ('M-07 trailing garbage accepted in integers', CF, 'if (end != v.c_str() + v.size() || errno == ERANGE) return "\'" + v + "\' is not a valid integer";', 'if (errno == ERANGE) return "overflow";'),
 ('M-08 base seed not wired into the VNS', CF, 'o.base_seed = c.base.seed.base_seed;', ''),
 ('M-09 n1_repeats not wired', CF, 'o.n1_repeats = c.base.neighbourhood.n1_repeats;', ''),
 ('M-10 record_trace not wired', CF, 'o.record_trace = c.output.record_trace;', ''),
 ('M-11 experiment.runs not validated', CF, 'if (c.experiment.runs < 1) e.push_back("experiment.runs must be >= 1");', ''),
 ('M-12 canonical dump omits a key', CF, 'for (const auto& f : config_fields()) s += f.key + " = " + f.get(c) + "\\n";', 'for (const auto& f : config_fields()) if (f.key != "n4.max_reanchor_variants") s += f.key + " = " + f.get(c) + "\\n";'),
 ('M-13 default TSP seed 43', CF, 'tsp_heuristic_seed = 42;', 'tsp_heuristic_seed = 43;'),
 ('M-14 default TSP restarts 9', CF, 'tsp_heuristic_restarts = 8;', 'tsp_heuristic_restarts = 9;'),
 ('M-15 default TSP exact limit 20', CF, 'tsp_max_exact_n = 21;', 'tsp_max_exact_n = 20;'),
 ('M-16 reference setter swapped', CF, 'if (v == "incumbent") c.vns.reference = AcceptanceReference::Incumbent;', 'if (v == "incumbent") c.vns.reference = AcceptanceReference::ShakenState;'),
 ('M-17 operator list loses its last entry', CF, 'c.vns.neighbourhoods = list;', 'c.vns.neighbourhoods = std::vector<OperatorId>(list.begin(), list.end() - 1);'),
 ('M-18 unknown keys accepted', CF, 'if (!f) return "unknown key \'" + key + "\'";', 'if (!f) return "";'),
 ('M-21 first_run_index ignored', RH, 'o.run_index = cfg.experiment.first_run_index + static_cast<std::uint64_t>(r);', 'o.run_index = static_cast<std::uint64_t>(r);'),
 ('M-22 dataset offset ignored', RH, 'cfg.base.instance.dataset_index + d', 'd'),
 ('M-23 adjacency window hard-coded', RH, 'init.flp_data, idx, sc.neighbourhood.adjacency_window};', 'init.flp_data, idx, 3};'),
 ('M-24 TSP seed hard-coded', RH, 'cfg.initial.tsp_heuristic_seed);', '42);'),
 ('M-25 k-means policy not passed', RH, 'pol.kmeans_policy = cfg.initial.kmeans;', ''),
 ('M-26 objective check dropped in verification', RH, 'if (v.objective != stated_objective) return "evaluator objective differs from the stated objective";', ''),
 ('M-27 exactly-once check weakened', RH, 'if (it == count.end() || it->second != 1)', 'if (it == count.end())'),
 ('M-28 verified flag always true', RH, 'R.verified = R.verify_note.empty();', 'R.verified = true;'),
 ('M-29 dataset index not applied to the instance', RH, 'sc.instance.dataset_index = dataset_index;', ''),
 ('M-31 improvement percent uses the wrong base', WH, '100.0 * (v.initial_objective - v.best_objective) / v.initial_objective', '100.0 * (v.initial_objective - v.best_objective) / v.best_objective'),
 ('M-32 minutes = seconds / 3600', WH, 'seconds / 60.0', 'seconds / 3600.0'),
 ('M-33 guard trigger not reported', WH, 'j.field("guard_triggered", v.guard_triggered());', 'j.field("guard_triggered", false);'),
 ('M-34 deterministic flag constant', WH, 'j.field("deterministic", !(cfg.vns.max_seconds > 0.0));', 'j.field("deterministic", true);'),
 ('M-35 timing.json digested', WH, 'if (f.deterministic) j.field("fnv1a64", fnv1a64_hex(f.content)); else j.field("fnv1a64", "not-reproducible");', 'j.field("fnv1a64", fnv1a64_hex(f.content));'),
 ('M-36 overwrite protection off', WH, 'if (fs::exists(root / "results.json", ec) && !overwrite) {', 'if (false) {'),
 ('M-37 stale traces.csv kept', WH, 'fs::remove(root / "traces.csv", ec);', ''),
 ('M-38 best solutions not written', WH, 'if (cfg.output.write_solutions) { j.key("best_solution"); write_solution(j, v.best); }', ''),
 ('M-39 JSON quote not escaped', WH, 'case \'"\': s_ += "\\\\\\""; break;', 'case \'"\': s_ += "\\""; break;'),
 ('M-40 csv loses a column', WH, 'd(v.final_temperature) + "," + std::to_string(v.best.drone_trips.size())', 'std::to_string(v.best.drone_trips.size())'),
 ('M-41 std uses n instead of n - 1', WH, 'std::sqrt(ss / static_cast<double>(cnt - 1))', 'std::sqrt(ss / static_cast<double>(cnt))'),
 ('M-42 best run index not tracked', WH, 'best_run = static_cast<std::size_t>(R.run_index); }', '}'),
 ('M-43 non-finite doubles written as numbers', WH, 'if (!std::isfinite(v)) { s_ += "null"; return; }', ''),
 ('M-44 digest of a different content', WH, 'for (unsigned char c : s) { h ^= c; h *= 1099511628211ULL; }', 'for (unsigned char c : s) { h ^= c; h *= 1099511628213ULL; }'),
 ('M-47 output errors exit 1 instead of 4', CH, 'if (!W.ok) { err << "error: " << W.error << "\\n"; return 4; }', 'if (!W.ok) { err << "error: " << W.error << "\\n"; return 1; }'),
 ('M-49 --quiet ignored', CH, 'if (!o.quiet) progress =', 'if (true) progress ='),
 ('M-51 unknown arguments accepted', CH, 'else o.errors.push_back("unknown argument \'" + a + "\'");', 'else {}'),
]
only = sys.argv[1:]
for name, f, old, new in M:
    if only and not any(name.startswith(o) for o in only): continue
    shutil.rmtree(W, ignore_errors=True)
    os.makedirs(W)
    shutil.copytree(R + '/include', W + '/include')
    shutil.copytree(R + '/tests', W + '/tests')
    p = W + '/' + f
    s = open(p).read()
    allc = name.startswith('CW')
    if (s.count(old) != 1) and not allc:
        print(f'{name}: PATTERN COUNT {s.count(old)}', flush=True); continue
    open(p, 'w').write(s.replace(old, new))
    kills = []
    tests = tuple(os.environ.get('TDR_TESTS', 'test_experiment_config,test_runner_cli').split(','))
    procs = [(t, subprocess.Popen(['g++', '-std=c++20', '-O0', '-I' + W + '/include', '-I' + W + '/tests', '-o', W + '/' + t, W + '/tests/' + t + '.cpp', LIB], stdout=subprocess.PIPE, stderr=subprocess.PIPE)) for t in tests]
    for t, pr in procs:
        pr.communicate()
        if pr.returncode != 0:
            kills.append(t + ':COMPILE-ERROR'); continue
        try:
            r = subprocess.run([W + '/' + t], capture_output=True, text=True, timeout=60)
            if r.returncode != 0: kills.append(t)
        except subprocess.TimeoutExpired:
            kills.append(t + ':TIMEOUT')
    if any('COMPILE-ERROR' in k for k in kills): status = 'INVALID MUTANT (does not compile): ' + ', '.join(kills)
    else: status = 'KILLED by ' + ', '.join(kills) if kills else 'SURVIVED'
    print(f'{name}: {status}', flush=True)
