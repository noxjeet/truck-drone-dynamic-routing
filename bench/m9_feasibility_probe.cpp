// M9 read-only diagnostic (NOT part of the default build or of CTest; cmake -DTDR_BUILD_BENCH=ON): why are the trip-based
// neighbourhoods rarely applicable and N7 mostly infeasible? For one configuration (default: config/m9/n10_seed1.cfg) and every dataset
// it takes two base solutions -- the initial solution and the best solution of VNS run 0 (identical to run_index 0 of tdr_static) --
// and sweeps every operator over EVERY anchor (all truck customers for N7, all trips for the others), tabulating the evaluator
// outcome by reason. It changes no behaviour: operators, evaluator and options are the production ones; only N7's adjacency window is
// also tried at a larger value to show that the window is not what limits feasibility.
//
//   m9_feasibility_probe CONFIG.cfg [num_datasets]

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "config/experiment_config.hpp"
#include "flp/flp_arc_index.hpp"
#include "graph/spatial_network.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"
#include "search/neighbourhood_n1.hpp"
#include "search/neighbourhood_n2.hpp"
#include "search/neighbourhood_n3.hpp"
#include "search/neighbourhood_n5.hpp"
#include "search/neighbourhood_n6.hpp"
#include "search/neighbourhood_n7.hpp"
#include "search/neighbourhood_n8.hpp"
#include "search/vns.hpp"
#include "tsp/tsp_solver_factory.hpp"

using namespace tdr;
using namespace tdr::search;

namespace {
struct Tally {
    std::size_t calls = 0, anchored = 0, generated = 0, evaluated = 0, accepted = 0, improving = 0;
    std::array<std::size_t, kRejectReasonCount> rej{};
    double best_rel = 1e300; // smallest (best objective - base) / base over anchors with a feasible candidate, in %
    void add(const OperatorResult& r) {
        ++calls;
        if (r.status == OperatorStatus::Ok) ++anchored;
        generated += r.generated; evaluated += r.evaluated; accepted += r.accepted;
        for (std::size_t i = 0; i < kRejectReasonCount; ++i) rej[i] += r.rejected[i];
        if (r.improves()) ++improving;
        if (r.best) best_rel = std::min(best_rel, 100.0 * (r.best->objective - r.base_objective) / r.base_objective);
    }
    void print(const std::string& label) const {
        std::cout << "    " << std::left << std::setw(24) << label << std::right << " anchors " << std::setw(3) << anchored << "/" << std::setw(3) << calls
                  << "  generated " << std::setw(7) << generated << "  feasible " << std::setw(6) << accepted << "  improving anchors " << std::setw(3) << improving;
        if (accepted) std::cout << "  best feasible vs base " << std::showpos << std::fixed << std::setprecision(2) << best_rel << "%" << std::noshowpos;
        std::cout << "  rejected:";
        bool any = false;
        for (std::size_t i = 1; i < kRejectReasonCount; ++i) if (rej[i]) { std::cout << " " << reject_reason_name(static_cast<RejectReason>(i)) << "=" << rej[i]; any = true; }
        if (!any) std::cout << " none";
        std::cout << "\n";
    }
};
} // namespace

int main(int argc, char** argv) {
    if (argc < 2) { std::cerr << "usage: m9_feasibility_probe CONFIG.cfg [num_datasets]\n"; return 2; }
    config::ExperimentConfig cfg;
    const auto lr = config::load_config_file(argv[1], cfg);
    if (!lr.errors.empty()) { for (const auto& e : lr.errors) std::cerr << e << "\n"; return 2; }
    const int nds = argc > 2 ? std::stoi(argv[2]) : cfg.experiment.num_datasets;
    std::cout << "m9_feasibility_probe  config " << argv[1] << "  customers " << cfg.base.instance.num_customers << "  endurance " << cfg.base.vehicle.endurance_s
              << " s  window " << cfg.base.neighbourhood.adjacency_window << "\n";
    for (int d = 0; d < nds; ++d) {
        config::StaticConfig sc = cfg.base;
        sc.instance.dataset_index = cfg.base.instance.dataset_index + d;
        const auto gen = instance::generate_static_instance(sc);
        const auto& inst = gen.instance;
        init::InitialSolutionPolicy pol;
        pol.kmeans_policy = cfg.initial.kmeans;
        pol.flp_policy.unit_division_length = sc.flp.unit_division_length_m;
        const tsp::TSPSolverFactory factory(cfg.initial.tsp_max_exact_n, cfg.initial.tsp_heuristic_restarts, cfg.initial.tsp_heuristic_seed);
        const auto net0 = inst.make_network();
        const auto init = init::build_initial_solution(inst.depot, inst.customers, inst.customer_data, net0, pol, factory);
        const graph::SpatialNetwork net(init.node_registry, inst.truck, inst.drone);
        const flp::FlpArcIndex idx(init.flp_data);
        const SearchContext ctx{net, inst.customer_data, init.flp_data, idx, sc.neighbourhood.adjacency_window};
        VnsOptions vo = config::effective_vns_options(cfg);
        vo.run_index = cfg.experiment.first_run_index;
        const VnsResult vr = run_vns(init.solution, ctx, vo);

        std::cout << "dataset " << sc.instance.dataset_index << ": pool " << init.flp_data.size() << ", initial obj " << init.objective_value / 60.0 << " min (" << init.solution.drone_trips.size()
                  << " trips), VNS run " << vo.run_index << " best " << vr.best_objective / 60.0 << " min (" << vr.best.drone_trips.size() << " trips)\n";
        const std::vector<std::pair<std::string, const model::Solution*>> bases = {{"initial", &init.solution}, {"VNS best", &vr.best}};
        for (const auto& [bname, bp] : bases) {
            const model::Solution& base = *bp;
            const Skeleton sk = make_skeleton(base.truck_route, net.nodes());
            std::vector<model::NodeId> anchors7;
            for (std::size_t s = 1; s + 1 < sk.size(); ++s)
                if (net.nodes().at(sk.node[s]).type == model::NodeType::Customer) anchors7.push_back(sk.node[s]);
            std::cout << "  base = " << bname << " (" << anchors7.size() << " truck customers, " << base.drone_trips.size() << " trips)\n";
            for (int w : {sc.neighbourhood.adjacency_window, 1000}) {
                const SearchContext cw{net, inst.customer_data, init.flp_data, idx, w};
                Tally t;
                for (auto a : anchors7) t.add(n7_truck_node_delete_drone_insert(base, cw, a));
                t.print("N7 truck->drone w=" + std::to_string(w));
            }
            Tally t1, t2, t3, t5, t6, t8;
            for (std::size_t i = 0; i < base.drone_trips.size(); ++i) {
                t1.add(n1_launch_node_relocate(base, ctx, i)); t2.add(n2_recover_node_relocate(base, ctx, i)); t3.add(n3_drone_service_node_relocate(base, ctx, i));
                t5.add(n5_td_service_node_swap(base, ctx, i)); t6.add(n6_drone_drone_swap(base, ctx, i)); t8.add(n8_drone_node_delete_truck_insert(base, ctx, i));
            }
            if (!base.drone_trips.empty()) {
                t1.print("N1 launch relocate"); t2.print("N2 recovery relocate"); t3.print("N3 drone-node relocate");
                t5.print("N5 truck/drone swap"); t6.print("N6 drone/drone swap"); t8.print("N8 drone->truck");
            } else std::cout << "    (no trips: N1, N2, N3, N5, N6, N8 have no anchor by construction)\n";
        }
    }
    return 0;
}
