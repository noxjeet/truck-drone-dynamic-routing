import os, shutil, subprocess, sys
# M7 mutation harness: injects one bug at a time into a COPY of include/ and tests/ and checks that the M7 tests (test_vns, test_vns_oracle) fail.
# usage: TDR_REPO=<repo> TDR_LIB=<path of libtdr_core.a from a Release build> TDR_WORK=<scratch dir> python3 m7_mutate.py [name-prefix ...]
# The repository is never modified. 'CW-*' mutants are applied to every occurrence of their pattern.
R = os.environ.get('TDR_REPO', os.getcwd())
W = os.environ.get('TDR_WORK', '/tmp/tdr_m7_mut') + '/w'
LIB = os.environ['TDR_LIB']

VH = 'include/search/vns.hpp'
M = [
 ('V-01 loop runs iter_max - 1 passes', VH, 'iter <= o.iter_max; ++iter', 'iter < o.iter_max; ++iter'),
 ('V-02 loop runs iter_max + 1 passes', VH, 'iter <= o.iter_max; ++iter', 'iter <= o.iter_max + 1; ++iter'),
 ('V-03 iterations not recorded', VH, '        R.iterations = iter;\n', ''),
 ('V-04 temperature never cooled', VH, '        T = std::max(o.t_min, o.cooling_rate * T);\n', ''),
 ('V-05 cooled twice per pass', VH, '        T = std::max(o.t_min, o.cooling_rate * T);\n', '        T = std::max(o.t_min, o.cooling_rate * T);\n        T = std::max(o.t_min, o.cooling_rate * T);\n'),
 ('V-06 temperature floor removed', VH, 'T = std::max(o.t_min, o.cooling_rate * T);', 'T = o.cooling_rate * T;'),
 ('V-07 absolute T0 ignored', VH, 'o.t0_absolute > 0.0 ? o.t0_absolute :', 'false ? o.t0_absolute :'),
 ('V-08 T0 not scaled by the initial objective', VH, 'o.t0_fraction * v0.objective', 'o.t0_fraction'),
 ('V-09 SA exponent sign flipped', VH, 'std::exp((ref - fz) / T)', 'std::exp((fz - ref) / T)'),
 ('V-10 SA test strict (<)', VH, 'if (u <= std::exp((ref - fz) / T))', 'if (u < std::exp((ref - fz) / T))'),
 ('V-11 acceptance draw not taken', VH, 'const double u = vns_rng.uniform01();', 'const double u = 0.5;'),
 ('V-12 improvement test non-strict (<=)', VH, 'if (fz < ref - o.min_improvement) {', 'if (fz <= ref - o.min_improvement) {'),
 ('V-13 min_improvement ignored', VH, 'if (fz < ref - o.min_improvement) {', 'if (fz < ref) {'),
 ('V-14 no restart on improvement', VH, 'k = improving ? 0 : k + 1;', 'k = k + 1;'),
 ('V-15 restart after SA acceptance too', VH, 'k = improving ? 0 : k + 1;', 'k = accepted ? 0 : k + 1;'),
 ('V-16 no advance after SA acceptance', VH, 'k = improving ? 0 : k + 1;', 'k = improving ? 0 : (accepted ? k : k + 1);'),
 ('V-17 accepted candidate not carried forward', VH, '                    w = std::move(r.best->solution);\n', ''),
 ('V-18 incumbent not updated on acceptance', VH, '                    if (o.reference == AcceptanceReference::Incumbent) { x = w; fx = fw; }\n', ''),
 ('V-19 best not updated on acceptance', VH, '{ x = w; fx = fw; }\n                    if (fw < fbest) { best = w; fbest = fw; ++R.best_updates; R.last_best_iteration = iter; best_improved = true; }', '{ x = w; fx = fw; }\n'),
 ('V-20 best not updated by the shaken solution', VH, '            if (fw < fbest) { best = w; fbest = fw; ++R.best_updates; R.last_best_iteration = iter; best_improved = true; }\n            std::size_t k = 0, steps = 0;', '            std::size_t k = 0, steps = 0;'),
 ('V-21 best stores the incumbent instead of the accepted solution', VH, '                    if (fw < fbest) { best = w; fbest = fw; ++R.best_updates; R.last_best_iteration = iter; best_improved = true; }\n                }\n                k =', '                    if (fw < fbest) { best = x; fbest = fw; ++R.best_updates; R.last_best_iteration = iter; best_improved = true; }\n                }\n                k ='),
 ('V-22 reference always f(x)', VH, 'o.reference == AcceptanceReference::Incumbent ? fx : fw;', 'fx;'),
 ('V-23 reference always f(w)', VH, 'o.reference == AcceptanceReference::Incumbent ? fx : fw;', 'fw;'),
 ('V-24 ShakenState: incumbent not set at the end of the pass', VH, '            if (o.reference == AcceptanceReference::ShakenState) { x = std::move(w); fx = fw; }\n', ''),
 ('V-25 not-applicable shake ends the attempts', VH, '{ ++R.shakes_not_applicable; continue; }', '{ ++R.shakes_not_applicable; break; }'),
 ('V-26 infeasible shake accepted', VH, '            if (!s.feasible) { ++R.shakes_infeasible; continue; }\n', ''),
 ('V-27 only one shake attempt', VH, 'a < o.max_shake_attempts', 'a < 1'),
 ('V-28 skipped passes not counted', VH, '            ++R.iterations_skipped;\n', ''),
 ('V-29 applied shakes not counted', VH, '            ++R.shakes_applied;\n', ''),
 ('V-30 applied shakes by kind not counted', VH, '            ++R.shakes_applied_by_kind[static_cast<std::size_t>(s.kind)];\n', ''),
 ('V-31 no-candidate calls not counted', VH, '{ ++st.no_candidate; ++k; continue; }', '{ ++k; continue; }'),
 ('V-32 operator status ignored', VH, 'if (r.status != OperatorStatus::Ok) { ++st.no_anchor; ++k; continue; }', ''),
 ('V-33 Vns stream seeded from the Shake stream', VH, 'tdr::util::SeedStream::Vns, o.run_index', 'tdr::util::SeedStream::Shake, o.run_index'),
 ('V-34 run index ignored in the Vns seed', VH, 'tdr::util::SeedStream::Vns, o.run_index', 'tdr::util::SeedStream::Vns, 0'),
 ('V-35 run index ignored in the Shake seed', VH, 'tdr::util::SeedStream::Shake, o.run_index));\n\n    tdr::model::Solution x', 'tdr::util::SeedStream::Shake, 0));\n\n    tdr::model::Solution x'),
 ('V-36 default list: N5 and N8 swapped', VH, 'OperatorId::N4, OperatorId::N5, OperatorId::N8,', 'OperatorId::N4, OperatorId::N8, OperatorId::N5,'),
 ('V-37 default list: N1 repeats ignored', VH, 'if (r < n1_repeats) v.push_back(OperatorId::N1);', 'if (r < 1) v.push_back(OperatorId::N1);'),
 ('V-38 operator mapping: N5 calls N6', VH, 'case OperatorId::N5: return n5_td_service_node_swap(w, ctx, rng);', 'case OperatorId::N5: return n6_drone_drone_swap(w, ctx, rng);'),
 ('V-39 N4 options ignored', VH, 'n4_truck_service_node_relocate(w, ctx, rng, o.n4)', 'n4_truck_service_node_relocate(w, ctx, rng)'),
 ('V-40 no-improvement stop one pass late', VH, 'since_best >= o.max_no_improve', 'since_best > o.max_no_improve'),
 ('V-41 no-improvement counter never reset', VH, 'since_best = best_improved ? 0 : since_best + 1;', 'since_best = since_best + 1;'),
 ('V-42 time limit ignored', VH, 'if (o.max_seconds > 0.0 && std::chrono', 'if (false && std::chrono'),
 ('V-43 local-step guard off', VH, 'if (o.max_local_steps > 0 && steps >= o.max_local_steps)', 'if (false)'),
 ('V-44 truncated passes not counted', VH, '++R.passes_truncated; truncated = true; break;', 'truncated = true; break;'),
 ('V-53 guard-truncated pass reported as Completed', VH, 'if (truncated) row.end = VnsPassEnd::GuardTruncated; else ++R.passes_completed;', 'if (truncated) row.end = VnsPassEnd::Completed; else ++R.passes_completed;'),
 ('V-54 completed passes not counted', VH, 'else ++R.passes_completed;', ';'),
 ('V-55 skipped pass reported as Completed', VH, '            row.end = VnsPassEnd::Skipped;\n', ''),
 ('V-56 total local steps not accumulated', VH, '            R.local_steps_total += steps;\n', ''),
 ('V-57 truncated flag never set', VH, '++R.passes_truncated; truncated = true; break;', '++R.passes_truncated; break;'),
 ('V-58 busiest-pass steps not tracked', VH, '            R.max_local_steps_in_pass = std::max(R.max_local_steps_in_pass, steps);\n', ''),
 ('V-45 infeasible initial solution accepted', VH, 'if (!v0.feasible) {', 'if (false) {'),
 ('V-46 invalid cooling rate not reported', VH, 'if (!(o.cooling_rate > 0.0 && o.cooling_rate <= 1.0)) e.push_back("cooling_rate must be in (0, 1]");', ''),
 ('V-47 trace temperature recorded as 0', VH, '        row.temperature = T;\n', ''),
 ('V-48 improving moves not counted per operator', VH, '                    ++st.improving;\n', ''),
 ('V-49 SA-accepted moves not counted', VH, '++st.sa_accepted; ++R.sa_accepted_moves;', '++R.sa_accepted_moves;'),
 ('V-50 best update bookkeeping skipped on acceptance (last_best_iteration)', VH, '{ best = w; fbest = fw; ++R.best_updates; R.last_best_iteration = iter; best_improved = true; }\n                }\n                k =', '{ best = w; fbest = fw; ++R.best_updates; best_improved = true; }\n                }\n                k ='),
 ('V-51 final current not returned', VH, '    R.current = std::move(x);\n', ''),
 ('V-52 improvement counted as best-improvement flag only when strictly better (best_improved set on every acceptance)', VH, 'if (fw < fbest) { best = w; fbest = fw; ++R.best_updates; R.last_best_iteration = iter; best_improved = true; }\n                }\n                k =', 'if (fw < fbest) { best = w; fbest = fw; ++R.best_updates; R.last_best_iteration = iter; } best_improved = true;\n                }\n                k ='),
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
    tests = tuple(os.environ.get('TDR_TESTS', 'test_vns,test_vns_oracle').split(','))
    procs = [(t, subprocess.Popen(['g++', '-std=c++20', '-O1', '-I' + W + '/include', '-I' + W + '/tests', '-o', W + '/' + t, W + '/tests/' + t + '.cpp', LIB], stdout=subprocess.PIPE, stderr=subprocess.PIPE)) for t in tests]
    for t, pr in procs:
        pr.communicate()
        if pr.returncode != 0:
            kills.append(t + ':COMPILE-ERROR'); continue
        try:
            r = subprocess.run([W + '/' + t], capture_output=True, text=True, timeout=25)
            if r.returncode != 0: kills.append(t)
        except subprocess.TimeoutExpired:
            kills.append(t + ':TIMEOUT')
    if any('COMPILE-ERROR' in k for k in kills): status = 'INVALID MUTANT (does not compile): ' + ', '.join(kills)
    else: status = 'KILLED by ' + ', '.join(kills) if kills else 'SURVIVED'
    print(f'{name}: {status}', flush=True)
