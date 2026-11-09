import os, shutil, subprocess, sys
# M2 mutation harness: injects one bug at a time into a COPY of include/ and tests/ and checks that the M2 tests fail.
# usage: TDR_REPO=<repo> TDR_LIB=<path of libtdr_core.a from a Release build> TDR_WORK=<scratch dir> python3 m2_mutate.py [name-prefix ...]
# The repository is never modified. 'CW-*' mutants are applied to every occurrence of their pattern.
R = os.environ.get('TDR_REPO', os.getcwd())
W = os.environ.get('TDR_WORK', '/tmp/tdr_m2_mut') + '/w'
LIB = os.environ['TDR_LIB']

N4 = 'include/search/neighbourhood_n4.hpp'
N5 = 'include/search/neighbourhood_n5.hpp'
RO = 'include/search/route_ops.hpp'
CW = 'include/search/candidate_window.hpp'
MT = 'include/search/move_types.hpp'

M = [
 ('N4-01 window +1 (relocate gaps)', N4, 'for (std::size_t g : gaps_within_window(sk, h_c, w))', 'for (std::size_t g : gaps_within_window(sk, h_c, w + 1))'),
 ('N4-02 window -1 (relocate gaps)', N4, 'for (std::size_t g : gaps_within_window(sk, h_c, w))', 'for (std::size_t g : gaps_within_window(sk, h_c, w - 1))'),
 ('N4-03 exchange window +1', N4, 'for (NodeId y : pi.neighbours(anchor, w)) {', 'for (NodeId y : pi.neighbours(anchor, w + 1)) {'),
 ('N4-04 no duplicate removal', N4, 'if (r == route) {', 'if (false && r == route) {'),
 ('N4-05 affected check skipped', N4, 'if (!foreign.empty()) {', 'if (false) {'),
 ('N4-06 incident gaps not excluded', N4, 'if (g + 1 == s_c || g == s_c) continue;', ''),
 ('N4-07 wrong gap mapping after removal', N4, 'const std::size_t g2 = (g > s_c) ? g - 1 : g;', 'const std::size_t g2 = g;'),
 ('N4-08 tie-break last wins (<=)', N4, 'if (v.feasible && (!res.best || v.objective < res.best->objective)) {', 'if (v.feasible && (!res.best || v.objective <= res.best->objective)) {'),
 ('N4-09 trips silently dissolved', N4, 'cand.drone_trips = base.drone_trips;', 'cand.drone_trips.clear();'),
 ('N4-10 affected rejection not counted (Reject policy)', N4, '                reject(RejectReason::AffectedTrips);\n', ''),
 ('N4-11 affected trip list empty', N4, 'for (std::size_t t : trips_launching_from(cand, f.flp)) affected.insert(t);', ''),
 ('N4-12 relocate lands before the gap FLPs (insertion pos -> gap start)', N4, 'r.insert(r.begin() + static_cast<std::ptrdiff_t>(*pos), anchor);', 'r.insert(r.begin() + static_cast<std::ptrdiff_t>(sk2.route_position[g2] + 1), anchor);'),
 ('N4-13 exchange partner order reversed', N4, 'for (NodeId y : pi.neighbours(anchor, w)) {', 'for (NodeId y : [&] { auto v = pi.neighbours(anchor, w); std::reverse(v.begin(), v.end()); return v; }()) {'),
 ('N4-14 evaluate_candidates ignored', N4, '} else if (opt.evaluate_candidates) {', '} else if (true) {'),
 # ---- N4 conservative re-anchoring (Reanchor policy) ----
 ('RA-01 recovery check skipped', N4, 'if (const auto bad = detail_n4::trips_with_invalid_recovery(cand); !bad.empty()) {', 'if (const auto bad = detail_n4::trips_with_invalid_recovery(cand); false) {'),
 ('RA-02 no-replacement check skipped', N4, 'if (!no_flp.empty()) {', 'if (false) {'),
 ('RA-03 drone id changed by re-anchoring', N4, 'var.drone_trips[stale[k].trip].launch_node = to;', 'var.drone_trips[stale[k].trip].launch_node = to; var.drone_trips[stale[k].trip].drone_id += 1;'),
 ('RA-04 recovery node changed by re-anchoring', N4, 'var.drone_trips[stale[k].trip].launch_node = to;', 'var.drone_trips[stale[k].trip].launch_node = to; var.drone_trips[stale[k].trip].recovery_node = to;'),
 ('RA-05 variant tie-break last wins (<=)', N4, 'if (!any_feasible || v.objective < best_objective) {', 'if (!any_feasible || v.objective <= best_objective) {'),
 ('RA-06 stale trips handled in reverse order', N4, 'return a.trip < b.trip; });', 'return a.trip > b.trip; });'),
 ('RA-07 enumeration skips the last option', N4, 'if (++choice[k] < stale[k].options->size()) break;', 'if (++choice[k] + 1 < stale[k].options->size()) break;'),
 ('RA-08 stale FLP not removed from the route', N4, 'if (!is_stale) r.push_back(v);', 'r.push_back(v);'),
 ('RA-09 cap_reached off by one (>=)', N4, 'rec.cap_reached = combinations > budget;', 'rec.cap_reached = combinations >= budget;'),
 ('CAP-01 default budget back to 10000', N4, 'std::size_t max_reanchor_variants = 100;', 'std::size_t max_reanchor_variants = 10000;'),
 ('CAP-02 budget off by one (one extra combination)', N4, 'if (rec.variants_generated >= budget) break;', 'if (rec.variants_generated > budget) break;'),
 ('CAP-03 cap ignored (always exhaustive)', N4, 'const std::size_t budget = opt.max_reanchor_variants == 0 ? kMax : opt.max_reanchor_variants;', 'const std::size_t budget = kMax;'),
 ('CAP-04 cap_reached candidates not counted', N4, 'if (rec.cap_reached) ++res.cap_reached_candidates;', ''),
 ('CAP-05 cap 0 means zero budget instead of unlimited', N4, 'const std::size_t budget = opt.max_reanchor_variants == 0 ? kMax : opt.max_reanchor_variants;', 'const std::size_t budget = opt.max_reanchor_variants;'),
 ('CAP-06 enumeration order: FIRST stale trip fastest', N4, '''            std::size_t k = stale.size();
            while (k > 0) {
                --k;
                if (++choice[k] < stale[k].options->size()) break;
                choice[k] = 0;
                if (k == 0) { k = stale.size() + 1; break; }
            }
            if (k == stale.size() + 1) break;''', '''            std::size_t k = 0;
            while (k < stale.size()) {
                if (++choice[k] < stale[k].options->size()) break;
                choice[k] = 0;
                ++k;
            }
            if (k == stale.size()) break;'''),
 ('CAP-07 combinations_total not recorded', N4, 'rec.combinations_total = combinations;', ''),
 ('CAP-08 budget applied to the Reject policy path too (candidates dropped)', N4, 'rec.cap_reached = combinations > budget;', 'rec.cap_reached = combinations > budget; if (rec.cap_reached && budget == 1) { reject(RejectReason::NoReplacementFlp); finish(); return; }'),
 ('RA-10 invalid-recovery counted twice', N4, '            rec.affected_trips = bad;\n            reject(RejectReason::InvalidRecovery);', '            rec.affected_trips = bad;\n            reject(RejectReason::InvalidRecovery); ++res.rejected[static_cast<std::size_t>(RejectReason::InvalidRecovery)];'),
 ('RA-11 unaffected trips treated as stale', N4, 'for (std::size_t t : trips_launching_from(cand, f.flp)) {\n                detail_n4::StaleTrip st;', 'for (std::size_t t = 0; t < cand.drone_trips.size(); ++t) {\n                detail_n4::StaleTrip st;'),
 ('RA-12 no-replacement reported as affected-trips', N4, 'reject(RejectReason::NoReplacementFlp);', 'reject(RejectReason::AffectedTrips);'),
 ('RA-13 trips_reanchored counts candidates not trips', N4, 'res.trips_reanchored += stale.size();', 'res.trips_reanchored += 1;'),
 ('RA-14 evaluator judges the stale candidate, not the variant', N4, 'const EvalVerdict v = run_evaluator(var, move, re);', 'const EvalVerdict v = run_evaluator(cand, move, re);'),
 ('RA-15 stale candidates not counted', N4, 'if (!foreign.empty()) ++res.stale_launch_candidates;', ''),
 ('RA-16 trip launch not updated to the replacement', N4, 'var.drone_trips[stale[k].trip].launch_node = to;', ''),
 ('RA-17 recovery searched from route start, not after launch', N4, 'find_route_position_from(s.truck_route, t.recovery_node, *lp + 1)', 'find_route_position_from(s.truck_route, t.recovery_node, 0)'),
 ('RA-18 replacement inserted into the first gap, not the stale FLP\'s arc', N4, 'if (sk.node[g] != stale[k].arc_from || sk.node[g + 1] != stale[k].arc_to) continue;', ''),
 ('RA-19 re-anchored candidate not counted as accepted', N4, '                ++res.accepted;\n                ++res.reanchored_candidates;', '                ++res.reanchored_candidates;'),
 ('RA-20 options in reverse ratio order', N4, 'st.options = &ctx.flp_index.flps_on_arc(f.actual_from, f.actual_to);', 'static thread_local std::vector<tdr::model::NodeId> rev; rev = ctx.flp_index.flps_on_arc(f.actual_from, f.actual_to); std::reverse(rev.begin(), rev.end()); st.options = &rev;'),
 ('RA-21 evaluate_candidates ignored (Reanchor, unaffected path)', N4, '        const auto judge_as_is = [&] {\n            if (opt.evaluate_candidates) {', '        const auto judge_as_is = [&] {\n            if (true) {'),
 ('N5-01 partner window +1', N5, 'for (NodeId t : pi.neighbours(d, w)) {', 'for (NodeId t : pi.neighbours(d, w + 1)) {'),
 ('N5-02 launch window +1', N5, 'gaps_within_window(sk0, 2 * static_cast<int>(s_t), w)', 'gaps_within_window(sk0, 2 * static_cast<int>(s_t), w + 1)'),
 ('N5-03 launch window -1', N5, 'gaps_within_window(sk0, 2 * static_cast<int>(s_t), w)', 'gaps_within_window(sk0, 2 * static_cast<int>(s_t), w - 1)'),
 ('N5-04 launch window centre shifted (+1 half-step)', N5, 'gaps_within_window(sk0, 2 * static_cast<int>(s_t), w)', 'gaps_within_window(sk0, 2 * static_cast<int>(s_t) + 2, w)'),
 ('N5-05 wrong recovery node (x instead of y)', N5, 'nt.recovery_node = y;', 'nt.recovery_node = x;'),
 ('N5-06 unused FLPs not dropped', N5, 'const std::vector<NodeId> dropped = drop_unused_flps(sol0, nodes);', 'const std::vector<NodeId> dropped;'),
 ('N5-07 affected check skipped', N5, 'if (!affected_set.empty()) {', 'if (false) {'),
 ('N5-08 trips recovering at t ignored', N5, 'for (std::size_t k : trips_recovering_at(sol0, t)) affected_set.insert(kept_index[k]);', ''),
 ('N5-09 foreign-FLP trips ignored', N5, 'for (std::size_t k : trips_launching_from(sol0, f.flp)) affected_set.insert(kept_index[k]);', ''),
 ('N5-10 free-drone search bypassed (always drone 0)', N5, 'const auto drone = lowest_free_drone_for_trip(cand, cand.drone_trips.size() - 1, ctx.num_drones());', 'const std::optional<int> drone = 0;'),
 ('N5-11 tie-break last wins (<=)', N5, 'if (!res.best || v.objective < res.best->objective) {', 'if (!res.best || v.objective <= res.best->objective) {'),
 ('N5-12 old trip not dissolved', N5, 'for (std::size_t i : kept_index) sol0.drone_trips.push_back(base.drone_trips[i]);', 'for (std::size_t i = 0; i < base.drone_trips.size(); ++i) sol0.drone_trips.push_back(base.drone_trips[i]);'),
 ('N5-13 role swap wrong (t stays)', N5, 'sol0.truck_route[*pt] = d;', 'sol0.truck_route[*pt] = t;'),
 ('N5-14 on-route FLP reuse disabled (duplicate insert)', N5, 'if (std::find(cand.truck_route.begin(), cand.truck_route.end(), f) == cand.truck_route.end()) {', 'if (true) {'),
 ('N5-15 NoFreeDrone not counted', N5, '            ++res.rejected[static_cast<std::size_t>(rec.reason)];\n                } else {', '                } else {'),
 ('RO-01 FLP inserted in reverse ratio order', RO, 'other->second.position_ratio < mine->second.position_ratio ||', 'other->second.position_ratio > mine->second.position_ratio ||'),
 ('RO-02 FLP insertion ignores existing FLPs', RO, '        pos = p + 1;\n    }\n    return pos;', '    }\n    return pos;'),
 ('CW-01 window boundary strict (<)', CW, 'if (std::abs(2 * static_cast<int>(g) + 1 - h) <= 2 * window) out.push_back(g);', 'if (std::abs(2 * static_cast<int>(g) + 1 - h) < 2 * window) out.push_back(g);'),
 ('CW-02 window boundary +1 half-step', CW, 'if (std::abs(2 * static_cast<int>(g) + 1 - h) <= 2 * window) out.push_back(g);', 'if (std::abs(2 * static_cast<int>(g) + 1 - h) <= 2 * window + 1) out.push_back(g);'),
 ('MT-01 trip-order reason disabled (cascade reported)', MT, 'if (has("not reachable in truck-route order")) { v.reason = RejectReason::EvalTripOrder; return v; }', ''),
 ('MT-02 fleet check before endurance swapped off', MT, 'if (e.max_simultaneous_drones > num_drones) { v.reason = RejectReason::EvalFleetCapacity; return v; }', ''),
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
    tests = ('test_n4_n5', 'test_n4_n5_oracle')
    procs = [(t, subprocess.Popen(['g++', '-std=c++20', '-O1', '-I' + W + '/include', '-I' + W + '/tests', '-o', W + '/' + t, W + '/tests/' + t + '.cpp', LIB], stdout=subprocess.PIPE, stderr=subprocess.PIPE)) for t in tests]
    for t, pr in procs:
        pr.communicate()
        if pr.returncode != 0:
            kills.append(t + ':COMPILE-ERROR'); continue
        r = subprocess.run([W + '/' + t], capture_output=True, text=True, timeout=300)
        if r.returncode != 0: kills.append(t)
    if any('COMPILE-ERROR' in k for k in kills): status = 'INVALID MUTANT (does not compile): ' + ', '.join(kills)
    else: status = 'KILLED by ' + ', '.join(kills) if kills else 'SURVIVED'
    print(f'{name}: {status}', flush=True)
