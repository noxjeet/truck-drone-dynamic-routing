#pragma once

// Dynamic extension (DM3): an installed plan = Solution + the evaluator result computed FROM THAT Solution IN THAT World, bound together
// in one immutable payload. There is no way to build an InstalledPlan whose cached evaluation belongs to another solution or world:
// the only factory evaluates internally, and the payload is const. Invariant: eval().feasible == true.

#include <cstring>
#include <memory>
#include <optional>
#include <string>

#include "dynamic/world.hpp"

namespace tdr::dynamic {

class InstalledPlan {
public:
    // nullopt (with a reason) when the solution is not evaluator-feasible in `world`.
    static std::optional<InstalledPlan> make(tdr::model::Solution solution, std::shared_ptr<const World> world, double installed_at,
                                             std::uint32_t version, std::string* why = nullptr) {
        auto data = std::make_shared<Data>();
        data->eval = world->evaluate(solution);
        if (!data->eval.feasible) {
            if (why) {
                *why = "evaluator-infeasible";
                for (const auto& r : data->eval.violation_reasons) { *why += "; " + r; }
            }
            return std::nullopt;
        }
        data->solution = std::move(solution);
        InstalledPlan p;
        p.data_ = std::move(data);
        p.world_ = std::move(world);
        p.installed_at_ = installed_at;
        p.version_ = version;
        return p;
    }

    const tdr::model::Solution& solution() const { return data_->solution; }
    const tdr::timing::SolutionEvaluationResult& eval() const { return data_->eval; }
    const World& world() const { return *world_; }
    const std::shared_ptr<const World>& world_ptr() const { return world_; }
    double objective() const { return data_->eval.objective_value; }
    double installed_at() const { return installed_at_; }
    std::uint32_t version() const { return version_; }

    // Re-evaluates from scratch and compares bitwise with the cache (used by tests and the invariant checker).
    bool consistent() const {
        const auto e = world_->evaluate(data_->solution);
        auto same = [](double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; };
        if (!e.feasible || !same(e.objective_value, data_->eval.objective_value)) return false;
        if (e.truck_timing.arrival_time.size() != data_->eval.truck_timing.arrival_time.size()) return false;
        for (std::size_t i = 0; i < e.truck_timing.arrival_time.size(); ++i)
            if (!same(e.truck_timing.arrival_time[i], data_->eval.truck_timing.arrival_time[i]) ||
                !same(e.truck_timing.departure_time[i], data_->eval.truck_timing.departure_time[i])) return false;
        return true;
    }

private:
    struct Data {
        tdr::model::Solution solution;
        tdr::timing::SolutionEvaluationResult eval;
    };
    InstalledPlan() = default;
    std::shared_ptr<const Data> data_;
    std::shared_ptr<const World> world_;
    double installed_at_ = 0.0;
    std::uint32_t version_ = 0;
};

} // namespace tdr::dynamic
