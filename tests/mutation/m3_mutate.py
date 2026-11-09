import os, shutil, subprocess, sys
# M3 mutation harness: injects one bug at a time into a COPY of include/ and tests/ and checks that the M3 tests (test_n7_n8, test_n7_n8_oracle) fail.
# usage: TDR_REPO=<repo> TDR_LIB=<path of libtdr_core.a from a Release build> TDR_WORK=<scratch dir> python3 m3_mutate.py [name-prefix ...]
# The repository is never modified. 'CW-*' mutants are applied to every occurrence of their pattern.
R = os.environ.get('TDR_REPO', os.getcwd())
W = os.environ.get('TDR_WORK', '/tmp/tdr_m3_mut') + '/w'
LIB = os.environ['TDR_LIB']

N7 = 'include/search/neighbourhood_n7.hpp'
N8 = 'include/search/neighbourhood_n8.hpp'
RO = 'include/search/route_ops.hpp'
CW = 'include/search/candidate_window.hpp'

M = [
 # ---------------- N7 ----------------
 ('N7-01 window +1', N7, 'gaps_within_window_after_removal(sk0, s_t, w)', 'gaps_within_window_after_removal(sk0, s_t, w + 1)'),
 ('N7-02 window -1', N7, 'gaps_within_window_after_removal(sk0, s_t, w)', 'gaps_within_window_after_removal(sk0, s_t, w - 1)'),
 ('N7-03 window boundary strict (<)', N7, 'if (std::labs(h - 2 * static_cast<long>(s_removed)) <= 2L * w) out.push_back(g);', 'if (std::labs(h - 2 * static_cast<long>(s_removed)) < 2L * w) out.push_back(g);'),
 ('N7-04 merged-arc coordinate shifted (+2)', N7, 'else if (g + 1 == s_removed) h = 2 * static_cast<long>(s_removed);', 'else if (g + 1 == s_removed) h = 2 * static_cast<long>(s_removed) + 2;'),
 ('N7-05 left-of-removed coordinate wrong (+2)', N7, 'if (g + 1 < s_removed) h = 2 * static_cast<long>(g) + 1;', 'if (g + 1 < s_removed) h = 2 * static_cast<long>(g) + 3;'),
 ('N7-06 right-of-removed coordinate wrong (+1 instead of +3)', N7, 'long h = 2 * static_cast<long>(g) + 3;', 'long h = 2 * static_cast<long>(g) + 1;'),
 ('N7-07 t not removed from the route', N7, '    remove_from_route(sol0.truck_route, anchor);\n', ''),
 ('N7-08 wrong recovery node (x instead of y)', N7, 'nt.recovery_node = y;', 'nt.recovery_node = x;'),
 ('N7-09 affected check skipped', N7, 'if (!affected_set.empty()) {', 'if (false) {'),
 ('N7-10 trips recovering at t ignored', N7, '    for (std::size_t k : trips_recovering_at(sol0, anchor)) affected_set.insert(k);\n', ''),
 ('N7-11 foreign-FLP trips ignored', N7, '        for (std::size_t k : trips_launching_from(sol0, f.flp)) affected_set.insert(k);\n', ''),
 ('N7-12 free-drone search bypassed (always drone 0)', N7, 'const auto drone = lowest_free_drone_for_trip(cand, cand.drone_trips.size() - 1, ctx.num_drones());', 'const std::optional<int> drone = 0;'),
 ('N7-13 tie-break last wins (<=)', N7, 'if (!res.best || v.objective < res.best->objective) {', 'if (!res.best || v.objective <= res.best->objective) {'),
 ('N7-14 NoReplacementFlp not counted', N7, '        ++res.rejected[static_cast<std::size_t>(RejectReason::NoReplacementFlp)];\n', ''),
 ('N7-15 on-route FLP reuse disabled (duplicate insert)', N7, 'if (std::find(cand.truck_route.begin(), cand.truck_route.end(), f) == cand.truck_route.end()) {', 'if (true) {'),
 ('N7-16 NoFreeDrone not counted', N7, '                rec.reason = RejectReason::NoFreeDrone;\n                ++res.rejected[static_cast<std::size_t>(rec.reason)];', '                rec.reason = RejectReason::NoFreeDrone;'),
 ('N7-17 anchor validity check skipped', N7, 'if (std::find(truck_customers.begin(), truck_customers.end(), anchor) == truck_customers.end()) {', 'if (false) {'),
 ('N7-18 evaluate_candidates ignored', N7, '                if (opt.evaluate_candidates) {\n                    const auto e', '                if (true) {\n                    const auto e'),
 ('N7-19 gaps enumerated in reverse', N7, '    for (std::size_t g : gaps) {\n        const NodeId x = sk0.node[g], y = sk0.node[g + 1];', '    for (std::size_t g : std::vector<std::size_t>(gaps.rbegin(), gaps.rend())) {\n        const NodeId x = sk0.node[g], y = sk0.node[g + 1];'),
 ('N7-20 FLPs of an arc in reverse ratio order', N7, 'for (NodeId f : ctx.flp_index.flps_on_arc(x, y)) {', 'for (NodeId f : [&] { auto v = ctx.flp_index.flps_on_arc(x, y); std::reverse(v.begin(), v.end()); return v; }()) {'),
 ('N7-21 FLP inserted ignoring the ratio order', N7, 'const auto pos = flp_insertion_position(cand.truck_route, sk0, g, f, ctx.flp_data);', 'const auto pos = insertion_position_for_gap(sk0, g);'),
 ('N7-22 best move lacks the drone id', N7, '                m.drone_id = *drone;\n', ''),
 ('N7-23 NoReplacement test ignores the window (all gaps)', N7, '    for (std::size_t g : gaps) if (!ctx.flp_index.flps_on_arc(sk0.node[g], sk0.node[g + 1]).empty()) any_option = true;', '    for (std::size_t g = 0; g < sk0.gap_count(); ++g) if (!ctx.flp_index.flps_on_arc(sk0.node[g], sk0.node[g + 1]).empty()) any_option = true;'),
 ('N7-24 Rng overload always picks the first customer', N7, 'customers[rng.uniform_index(customers.size())]', 'customers[0]'),
 ('N7-25 original trips lost', N7, '    sol0.drone_trips = base.drone_trips;\n', ''),
 ('N7-26 new trip customer wrong (launch FLP instead of t)', N7, 'nt.service_customer = anchor;', 'nt.service_customer = f;'),
 ('N7-27 affected list not recorded', N7, 'rec.affected_trips.assign(affected_set.begin(), affected_set.end());', ''),
 ('N7-28 evaluator reason overridden to EvalOther', N7, '                    rec.reason = v.reason;\n                    if (v.feasible) {', '                    rec.reason = v.feasible ? v.reason : RejectReason::EvalOther;\n                    if (v.feasible) {'),
 # ---------------- N8 ----------------
 ('N8-01 default restricted to the D1 window (violates D5)', N8, 'for (std::size_t g = 0; g < sk.gap_count(); ++g) gaps.push_back(g);', '{ const PositionIndex pi(base, nodes); if (const auto h = pi.half_step(c)) gaps = gaps_within_window(sk, *h, ctx.adjacency_window); }'),
 ('N8-02 first gap skipped', N8, 'for (std::size_t g = 0; g < sk.gap_count(); ++g) gaps.push_back(g);', 'for (std::size_t g = 1; g < sk.gap_count(); ++g) gaps.push_back(g);'),
 ('N8-03 last gap skipped', N8, 'for (std::size_t g = 0; g < sk.gap_count(); ++g) gaps.push_back(g);', 'for (std::size_t g = 0; g + 1 < sk.gap_count(); ++g) gaps.push_back(g);'),
 ('N8-04 affected check skipped', N8, 'if (!affected.empty()) {', 'if (false) {'),
 ('N8-05 trip not dissolved', N8, '        if (i == trip_index) continue;\n', ''),
 ('N8-06 unused FLPs not dropped', N8, 'const std::vector<NodeId> dropped = drop_unused_flps(sol0, nodes);', 'const std::vector<NodeId> dropped;'),
 ('N8-07 customer inserted at the gap start (before its FLPs)', N8, 'const auto pos = insertion_position_for_gap(sk, g);', 'const std::optional<std::size_t> pos = sk.route_position[g] + 1;'),
 ('N8-08 tie-break last wins (<=)', N8, 'if (!res.best || v.objective < res.best->objective) {', 'if (!res.best || v.objective <= res.best->objective) {'),
 ('N8-09 insertion_window +1', N8, 'gaps_within_window(sk, *h, *opt.insertion_window)', 'gaps_within_window(sk, *h, *opt.insertion_window + 1)'),
 ('N8-10 insertion_window -1', N8, 'gaps_within_window(sk, *h, *opt.insertion_window)', 'gaps_within_window(sk, *h, *opt.insertion_window - 1)'),
 ('N8-11 insertion_window centre shifted (+2 half-steps)', N8, 'gaps_within_window(sk, *h, *opt.insertion_window)', 'gaps_within_window(sk, *h + 2, *opt.insertion_window)'),
 ('N8-12 affected trips reported with local, not base, indices', N8, 'affected.push_back(kept_index[k]);', 'affected.push_back(k);'),
 ('N8-13 affected list not sorted', N8, '        std::sort(affected.begin(), affected.end());\n', ''),
 ('N8-14 AffectedTrips not counted', N8, '            ++res.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)];\n', ''),
 ('N8-15 evaluate_candidates ignored', N8, '} else if (opt.evaluate_candidates) {', '} else if (true) {'),
 ('N8-16 flps_dropped not recorded', N8, '        rec.flps_dropped = dropped;\n', ''),
 ('N8-17 trip-index range check skipped', N8, 'if (trip_index >= base.drone_trips.size()) {', 'if (false) {'),
 ('N8-18 Rng overload always picks the first trip', N8, 'rng.uniform_index(base.drone_trips.size())', '0'),
 ('N8-19 evaluator rejection counted twice', N8, '                ++res.rejected[static_cast<std::size_t>(v.reason)];\n            }\n        }\n        if (opt.record_candidates) {', '                ++res.rejected[static_cast<std::size_t>(v.reason)]; ++res.rejected[static_cast<std::size_t>(v.reason)];\n            }\n        }\n        if (opt.record_candidates) {'),
 ('N8-20 best move lacks the replaced trip', N8, '                    best.move = m;\n', '                    best.move = m; best.move.replaced_trip = kNoIndex;\n'),
 ('N8-21 customer inserted twice (not removed first: duplicate coverage)', N8, 'cand.truck_route.insert(cand.truck_route.begin() + static_cast<std::ptrdiff_t>(*pos), c);', 'cand.truck_route.insert(cand.truck_route.begin() + static_cast<std::ptrdiff_t>(*pos), c); cand.truck_route.insert(cand.truck_route.begin() + static_cast<std::ptrdiff_t>(*pos), c);'),
 # ---------------- shared helpers (re-checked against the M3 tests) ----------------
 ('RO-01 FLP inserted in reverse ratio order', RO, 'other->second.position_ratio < mine->second.position_ratio ||', 'other->second.position_ratio > mine->second.position_ratio ||'),
 ('RO-02 FLP insertion ignores existing FLPs', RO, '        pos = p + 1;\n    }\n    return pos;', '    }\n    return pos;'),
 ('CW-01 window boundary strict (<)', CW, 'if (std::abs(2 * static_cast<int>(g) + 1 - h) <= 2 * window) out.push_back(g);', 'if (std::abs(2 * static_cast<int>(g) + 1 - h) < 2 * window) out.push_back(g);'),
 ('CW-02 window boundary +1 half-step', CW, 'if (std::abs(2 * static_cast<int>(g) + 1 - h) <= 2 * window) out.push_back(g);', 'if (std::abs(2 * static_cast<int>(g) + 1 - h) <= 2 * window + 1) out.push_back(g);'),
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
    tests = ('test_n7_n8', 'test_n7_n8_oracle')
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
