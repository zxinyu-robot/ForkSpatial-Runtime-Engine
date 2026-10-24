#include "contract/spatial_state.h"

namespace fork_spatial::contract {

bool same_generation(const AssetRef& a, const AssetRef& b) noexcept {
    return a.robot_id == b.robot_id && a.session_id == b.session_id &&
           a.asset_id == b.asset_id && a.frame_id == b.frame_id &&
           a.generation == b.generation;
}

bool validate(const AssetRef& ref, std::string& error) {
    if (ref.asset_id.empty()) {
        error = "asset_id empty";
        return false;
    }
    if (ref.frame_id.empty()) {
        error = "frame_id empty (no implicit world/map default)";
        return false;
    }
    return true;
}

DeltaDisposition dispose(const DeltaRef& delta, const AssetRef& local) noexcept {
    if (delta.asset_id != local.asset_id || delta.generation != local.generation) {
        return DeltaDisposition::StaleGeneration;
    }
    if (delta.target_revision <= local.revision) {
        return DeltaDisposition::AlreadyApplied;
    }
    if (delta.base_revision == local.revision) {
        return DeltaDisposition::Apply;
    }
    return DeltaDisposition::MissingDeltas;
}

std::string_view name(ReasonCode code) noexcept {
    switch (code) {
        case ReasonCode::None: return "none";
        case ReasonCode::NoWaypoints: return "no_waypoints";
        case ReasonCode::PlanFailed: return "plan_failed";
        case ReasonCode::PlanFailedRepeated: return "plan_failed_repeated";
        case ReasonCode::ReplanTimeout: return "replan_timeout";
        case ReasonCode::Stuck: return "stuck";
        case ReasonCode::LocalizationLost: return "localization_lost";
        case ReasonCode::LocalizationJump: return "localization_jump";
        case ReasonCode::RelocalizationTimeout: return "relocalization_timeout";
        case ReasonCode::FloorTransition: return "floor_transition";
        case ReasonCode::AssetStale: return "asset_stale";
        case ReasonCode::LinkLost: return "link_lost";
        case ReasonCode::SafetyVeto: return "safety_veto";
        case ReasonCode::ResyncRequired: return "resync_required";
        case ReasonCode::Timeout: return "timeout";
        case ReasonCode::Canceled: return "canceled";
    }
    return "unknown";
}

std::string_view name(LocalizationMode mode) noexcept {
    switch (mode) {
        case LocalizationMode::Normal3D: return "normal_3d";
        case LocalizationMode::Degraded2D: return "degraded_2d";
        case LocalizationMode::Relocalizing: return "relocalizing";
        case LocalizationMode::Verifying: return "verifying";
        case LocalizationMode::Hold: return "hold";
    }
    return "unknown";
}

ExecutionAuthority authority_for(LocalizationMode mode) noexcept {
    switch (mode) {
        case LocalizationMode::Normal3D: return ExecutionAuthority::Full;
        case LocalizationMode::Degraded2D: return ExecutionAuthority::Restricted;
        case LocalizationMode::Relocalizing:
        case LocalizationMode::Verifying: return ExecutionAuthority::Suspended;
        case LocalizationMode::Hold: return ExecutionAuthority::Revoked;
    }
    return ExecutionAuthority::Revoked;
}

ReasonCode admit(const SpatialStateHeader& header, const AssetRef& expected,
                 std::int64_t now_ns) noexcept {
    if (!same_generation(header.asset, expected)) {
        return ReasonCode::AssetStale;
    }
    if (header.stamp.expired_at(now_ns)) {
        return ReasonCode::AssetStale;
    }
    if (!header.health.usable()) {
        return header.health.reason != ReasonCode::None ? header.health.reason
                                                        : ReasonCode::LocalizationLost;
    }
    return ReasonCode::None;
}

}  // namespace fork_spatial::contract
