// 空间状态契约失效用例：定位重置 / 跨楼层 / 过期 Token / 地图版本不一致 /
// 规划失败 / 安全否决 / 执行超时。每项验证 Runtime 停止使用旧证据。
#include "contract/spatial_state.h"
#include "cognition/edge_policy.h"
#include "cognition/semantic_token.h"
#include "mission/mission_orchestrator.h"

#include <cstdlib>
#include <type_traits>
#include <iostream>

namespace {

using namespace fork_spatial::contract;
namespace cog = fork_spatial::cognition;

int failures = 0;

void expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

AssetRef floor(const char* asset, std::uint64_t revision, std::uint64_t generation) {
    return AssetRef{1, 7, asset, revision, generation, "map"};
}

cog::SemanticToken token(std::uint64_t id, const AssetRef& ref, std::int64_t ttl_ns) {
    cog::SemanticToken t;
    t.token_id = id;
    t.kind = cog::TokenKind::Frontier;
    t.resource = {ref.robot_id, ref.session_id, ref.asset_id, ref.revision, ref.frame_id,
                  ref.generation};
    t.stamp_ns = 1'000;
    t.ttl_ns = ttl_ns;
    t.confidence = 0.9;
    return t;
}

void case_localization_reset() {
    const auto gen1 = floor("b1/f2/map", 3, 1);
    const auto gen2 = floor("b1/f2/map", 3, 2);
    cog::TokenStore store;
    expect(store.upsert(token(1, gen1, 60'000'000'000)) == cog::UpsertCode::Inserted,
           "reset: token accepted");
    expect(store.upsert(token(2, gen2, 60'000'000'000)) == cog::UpsertCode::Inserted,
           "reset: new-gen token accepted");
    expect(store.invalidate_generation(gen2) == 1, "reset: TTL 未到的旧代 Token 必须作废");
    expect(store.valid_tokens().size() == 1 && store.valid_tokens()[0].token_id == 2,
           "reset: only current generation survives");
    expect(store.invalidate_generation(gen2) == 0, "reset: idempotent");

    SpatialStateHeader old_state{gen1, Stamp{1'000, 0}, {}};
    expect(admit(old_state, gen2, 2'000) == ReasonCode::AssetStale,
           "reset: old-generation state rejected as AssetStale");
}

void case_floor_transition() {
    const auto f2 = floor("b1/f2/map", 1, 5);
    const auto f3 = floor("b1/f3/map", 1, 6);
    expect(!same_generation(f2, f3), "floor: different asset is never same generation");
    LocalizationHealth h;
    h.mode = LocalizationMode::Hold;
    h.reason = ReasonCode::FloorTransition;
    SpatialStateHeader during{f3, Stamp{1'000, 0}, h};
    expect(admit(during, f3, 1'500) == ReasonCode::FloorTransition,
           "floor: Hold during transition surfaces FloorTransition");
    expect(h.authority() == ExecutionAuthority::Revoked, "floor: Hold revokes execution");
    expect(authority_for(LocalizationMode::Relocalizing) == ExecutionAuthority::Suspended &&
               authority_for(LocalizationMode::Verifying) == ExecutionAuthority::Suspended,
           "relocalizing/verifying only suspend, task is kept");
    expect(authority_for(LocalizationMode::Degraded2D) == ExecutionAuthority::Restricted,
           "degraded 2d keeps restricted authority");
}

void case_expired_token() {
    const auto ref = floor("b1/f2/map", 1, 1);
    Stamp stamp{1'000, 5'000};
    expect(!stamp.expired_at(4'999) && stamp.expired_at(5'001), "expiry boundary");
    SpatialStateHeader h{ref, stamp, {}};
    expect(admit(h, ref, 4'000) == ReasonCode::None, "fresh state admitted");
    expect(admit(h, ref, 6'000) == ReasonCode::AssetStale, "expired state rejected");
    Stamp forever{1'000, 0};
    expect(!forever.expired_at(1'000'000'000'000), "valid_until=0 never expires");
}

void case_map_revision_mismatch() {
    const auto r3 = floor("b1/f2/map", 3, 1);
    const auto r4 = floor("b1/f2/map", 4, 1);
    // revision 变化不等于代数切换：局部更新仍可继续（MotionSLAM 规划 §5.2）。
    expect(same_generation(r3, r4), "revision bump alone keeps generation");
    SpatialStateHeader h{r3, Stamp{1'000, 0}, {}};
    expect(admit(h, r4, 2'000) == ReasonCode::None, "older revision still admitted");

    // 但决策引用的版本与实际执行的版本必须可对账。
    cog::PolicyDecision d;
    d.policy_version = 9;
    d.context = r3;
    cog::DecisionOutcome o;
    o.decision = d;
    o.executed_policy_version = 9;
    o.executed_context = r4;
    expect(o.executed_context != o.decision.context, "evidence chain records revision drift");

    std::string err;
    AssetRef no_frame = r3;
    no_frame.frame_id.clear();
    expect(!validate(no_frame, err) && !err.empty(), "empty frame_id rejected");
    expect(validate(r3, err), "complete ref valid");
}

void case_plan_failed() {
    using fork_spatial::mission::FailureCode;
    static_assert(std::is_same_v<FailureCode, ReasonCode>, "mission FailureCode is ReasonCode");
    expect(name(FailureCode::PlanFailedRepeated) == "plan_failed_repeated", "stable name");
    expect(name(ReasonCode::PlanFailed) == "plan_failed", "single-shot plan failure name");
    cog::PolicyDecision d;
    d.fallback = true;
    d.degrade_reason = ReasonCode::PlanFailedRepeated;
    expect(d.fallback && d.degrade_reason != ReasonCode::None, "fallback carries reason");
}

void case_safety_veto() {
    cog::DecisionOutcome o;
    o.vetoed = true;
    o.failure_code = ReasonCode::SafetyVeto;
    expect(name(o.failure_code) == "safety_veto", "veto name");
    expect(!o.succeeded && o.vetoed, "veto is not success");
}

void case_timeout() {
    expect(name(ReasonCode::Timeout) == "timeout", "timeout name");
    expect(name(ReasonCode::RelocalizationTimeout) == "relocalization_timeout", "reloc timeout");
    LocalizationHealth h;
    h.mode = LocalizationMode::Relocalizing;
    h.reason = ReasonCode::None;
    SpatialStateHeader s{floor("b1/f2/map", 1, 1), Stamp{1'000, 0}, h};
    expect(admit(s, floor("b1/f2/map", 1, 1), 2'000) == ReasonCode::LocalizationLost,
           "unusable mode without reason defaults to LocalizationLost");
}

void case_delta_out_of_order() {
    const auto local = floor("b1/f2/map", 42, 1);
    DeltaRef d{"b1/f2/map", 42, 43, 1, "token_stream", 0};
    expect(dispose(d, local) == DeltaDisposition::Apply, "delta: base matches → apply");
    expect(dispose(d, floor("b1/f2/map", 40, 1)) == DeltaDisposition::MissingDeltas,
           "delta: local at 40 must not apply delta(base=42)");
    expect(dispose(d, floor("b1/f2/map", 43, 1)) == DeltaDisposition::AlreadyApplied,
           "delta: duplicate delivery is a no-op");
    expect(dispose(d, floor("b1/f2/map", 42, 2)) == DeltaDisposition::StaleGeneration,
           "delta: generation changed → discard and resync");
    expect(dispose(d, floor("b1/f3/map", 42, 1)) == DeltaDisposition::StaleGeneration,
           "delta: other asset → discard");
    expect(name(ReasonCode::ResyncRequired) == "resync_required", "resync name");
}

}  // namespace

int main() {
    case_localization_reset();
    case_floor_transition();
    case_expired_token();
    case_map_revision_mismatch();
    case_plan_failed();
    case_safety_veto();
    case_timeout();
    case_delta_out_of_order();
    if (failures == 0) {
        std::cout << "contract tests passed\n";
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}
