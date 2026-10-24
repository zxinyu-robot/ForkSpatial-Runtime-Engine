#include "mission/mission_orchestrator.h"

#include <cmath>
#include <deque>

namespace fork_spatial::mission {
namespace {

constexpr double kNsPerS = 1e9;
double to_s(Timestamp ns) { return static_cast<double>(ns) / kNsPerS; }


}  // namespace

class MissionOrchestrator::Core {
public:
    explicit Core(OrchestratorConfig c) : cfg(c) {}

    OrchestratorConfig cfg;
    MissionStatus status{MissionStatus::Idle};
    MissionRequest request;
    std::size_t idx{0};
    std::deque<MissionCommand> commands;
    std::vector<MissionEvent> events;

    std::deque<double> ready_ticks;      // WaitReady 窗口内收到状态的时刻
    double ready_since{-1.0};
    double mission_start_s{-1.0};

    bool dispatched{false};
    double dispatch_s{-1.0};
    double best_dist{1e18};
    double last_progress_s{-1.0};
    double plan_deadline_s{-1.0};

    int plan_fail_count{0};
    int redispatch_count{0};
    int recovery_count{0};
    double recovery_until_s{-1.0};
    std::string recovery_reason;

    double now{-1.0};

    void emit(std::string name, std::string detail = {}) {
        events.push_back({std::move(name), status, idx, std::move(detail),
                          static_cast<Timestamp>(now * kNsPerS)});
    }

    void queue(CommandType t, std::string reason = {}) {
        commands.push_back({t, idx, idx < request.subgoals.size() ? request.subgoals[idx] : Subgoal{},
                            std::move(reason)});
    }

    void dispatch() {
        dispatched = true;
        dispatch_s = now;
        plan_deadline_s = now + cfg.plan_ready_timeout_s;
        emit("subgoal_dispatched");
        queue(CommandType::DispatchSubgoal);
    }

    void to_recovering(std::string reason) {
        recovery_reason = std::move(reason);
        ++recovery_count;
        if (recovery_count > cfg.max_plan_fail_recovery) {
            status = MissionStatus::Failed;
            emit("mission_terminal", "recovery_budget_exhausted:" + recovery_reason);
            queue(CommandType::StopTrajectory, "failed");
            queue(CommandType::MissionTerminal, "failed:" + recovery_reason);
            return;
        }
        status = MissionStatus::Recovering;
        recovery_until_s = now + cfg.recovery_wait_s;
        dispatched = false;
        emit("plan_fail_hold", recovery_reason);
        queue(CommandType::RecoveryWait, recovery_reason);
    }

    void tick_wait_ready(const RobotState& st) {
        if (!st.localization_valid) {
            return;
        }
        ready_ticks.push_back(now);
        while (!ready_ticks.empty() && now - ready_ticks.front() > cfg.ready_window_s) {
            ready_ticks.pop_front();
        }
        if (ready_ticks.size() >= static_cast<std::size_t>(cfg.ready_min_messages)) {
            if (ready_since < 0) {
                ready_since = now;
            }
            if (now - ready_since >= cfg.ready_settle_s) {
                status = MissionStatus::Running;
                mission_start_s = now;
                emit("mission_started");
                dispatch();
            }
        } else {
            ready_since = -1.0;
        }
    }

    void tick_running(const RobotState& st, bool planner_ready) {
        if (request.timeout_s > 0 && now - mission_start_s > request.timeout_s) {
            status = MissionStatus::Failed;
            emit("mission_terminal", "timeout");
            queue(CommandType::StopTrajectory, "timeout");
            queue(CommandType::MissionTerminal, "failed:timeout");
            return;
        }
        if (!st.localization_valid || st.localization_jump) {
            emit("localization_lost");
            queue(CommandType::StopTrajectory, "localization_lost");
            to_recovering("localization_lost");
            return;
        }
        if (planner_ready) {
            plan_deadline_s = now + cfg.plan_ready_timeout_s;
        } else if (now > plan_deadline_s) {
            emit("replan_timeout");
            ++plan_fail_count;
            if (plan_fail_count >= cfg.replan_fail_threshold) {
                plan_fail_count = 0;
                emit("plan_fail_assess");
                to_recovering("replan_fail_threshold");
            }
            return;
        }

        const auto& sg = request.subgoals[idx];
        const double dist = std::hypot(st.x - sg.x, st.y - sg.y, st.z - sg.z);
        if (dist < best_dist - cfg.progress_dist_delta_m) {
            best_dist = dist;
            last_progress_s = now;
        }
        if (now - last_progress_s > cfg.stuck_timeout_s) {
            emit("stuck");
            if (redispatch_count < cfg.stuck_redispatch_max) {
                ++redispatch_count;
                dispatched = false;
                emit("stuck_redispatch");
                dispatch();
            } else {
                to_recovering("stuck");
            }
            return;
        }

        const bool reached = planner_ready && dispatched &&
            dist <= sg.reach_tolerance_m && st.speed_mps <= cfg.reach_stable_speed_mps &&
            now - dispatch_s >= cfg.min_execute_before_reach_s;
        if (reached) {
            emit("subgoal_reached");
            ++idx;
            if (idx >= request.subgoals.size()) {
                status = MissionStatus::Succeeded;
                emit("mission_terminal", "all_subgoals");
                queue(CommandType::StopTrajectory, "succeeded");
                queue(CommandType::MissionTerminal, "succeeded");
            } else {
                dispatched = false;
                best_dist = 1e18;
                last_progress_s = now;
                plan_fail_count = 0;
                redispatch_count = 0;
                dispatch();
            }
        }
    }
};

MissionOrchestrator::MissionOrchestrator(OrchestratorConfig config) : core_(new Core(config)) {}
MissionOrchestrator::~MissionOrchestrator() { delete core_; }

bool MissionOrchestrator::submit(MissionRequest request) {
    if (core_->status != MissionStatus::Idle) {
        return false;
    }
    if (request.subgoals.empty()) {
        core_->status = MissionStatus::Failed;
        core_->emit("mission_terminal", "no_waypoints");
        core_->queue(CommandType::MissionTerminal, "failed:no_waypoints");
        return false;
    }
    core_->request = std::move(request);
    core_->status = MissionStatus::WaitReady;
    core_->emit("mission_wait_ready");
    return true;
}

bool MissionOrchestrator::cancel(const std::string& reason) {
    if (core_->status == MissionStatus::Succeeded || core_->status == MissionStatus::Failed ||
        core_->status == MissionStatus::Canceled || core_->status == MissionStatus::Idle) {
        return false;
    }
    core_->status = MissionStatus::Canceled;
    core_->emit("mission_terminal", "canceled:" + reason);
    core_->queue(CommandType::StopTrajectory, "canceled:" + reason);
    core_->queue(CommandType::MissionTerminal, "canceled:" + reason);
    return true;
}

MissionCommand MissionOrchestrator::tick(const RobotState& state, bool planner_ready) {
    core_->now = to_s(state.stamp_ns);
    switch (core_->status) {
        case MissionStatus::WaitReady:
            core_->tick_wait_ready(state);
            break;
        case MissionStatus::Running:
            core_->tick_running(state, planner_ready);
            break;
        case MissionStatus::Recovering:
            if (core_->now >= core_->recovery_until_s) {
                if (core_->redispatch_count < core_->cfg.stuck_redispatch_max) {
                    ++core_->redispatch_count;
                    core_->status = MissionStatus::Running;
                    core_->emit("recovery_done", core_->recovery_reason);
                    core_->dispatch();
                } else {
                    core_->status = MissionStatus::Failed;
                    core_->emit("mission_terminal", "redispatch_budget_exhausted");
                    core_->queue(CommandType::StopTrajectory, "failed");
                    core_->queue(CommandType::MissionTerminal, "failed:recovery");
                }
            }
            break;
        default:
            break;
    }
    if (!core_->commands.empty()) {
        auto cmd = std::move(core_->commands.front());
        core_->commands.pop_front();
        return cmd;
    }
    return {};
}

void MissionOrchestrator::on_plan_failed() {
    ++core_->plan_fail_count;
    if (core_->status == MissionStatus::Running &&
        core_->plan_fail_count >= core_->cfg.replan_fail_threshold) {
        core_->plan_fail_count = 0;
        core_->emit("plan_fail_assess");
        core_->to_recovering("replan_fail_threshold");
    }
}

void MissionOrchestrator::on_planner_ready() { core_->plan_deadline_s = core_->now + core_->cfg.plan_ready_timeout_s; }

MissionStatus MissionOrchestrator::status() const { return core_->status; }
std::size_t MissionOrchestrator::current_subgoal() const { return core_->idx; }
const std::vector<MissionEvent>& MissionOrchestrator::events() const { return core_->events; }

}  // namespace fork_spatial::mission
