#include "cognition/behavior_memory.h"
#include "cognition/edge_policy.h"
#include "cognition/semantic_token.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

using namespace fork_spatial::cognition;

int failures = 0;

void expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

GeometricObservation make_observation(fork_spatial::runtime::Timestamp stamp) {
    GeometricObservation obs;
    obs.map = {1, 1, "occupancy", 3, "map"};
    obs.stamp_ns = stamp;
    obs.localization_valid = true;
    obs.localization_quality = 0.9;
    obs.occupancy_ratio = 0.2;
    obs.frontiers = {{1, 4.0, 0.0, 0.0, 10.0, 0.9}, {2, 20.0, 0.0, 0.0, 5.0, 0.8}};
    obs.goal = std::vector<double>{6.0, 0.0, 1.0};
    return obs;
}

void test_encoder_and_lifecycle() {
    RuleBasedTokenEncoder encoder;
    TokenStore store;
    const auto tokens = encoder.encode(make_observation(0));
    expect(tokens.size() == 5, "scene + risk + 2 frontiers + goal");
    for (const auto& t : tokens) {
        expect(store.upsert(t) == UpsertCode::Inserted, "insert encoded token");
    }
    expect(store.valid_tokens().size() == 5, "all tokens valid");
    const auto near = store.find(1001);
    const auto far = store.find(1002);
    expect(near && far && near->attributes.priority > far->attributes.priority,
           "closer high-gain frontier has higher priority");

    auto older = tokens.front();
    older.resource.revision = 2;
    expect(store.upsert(older) == UpsertCode::RejectedStaleRevision, "reject older map revision");

    auto invalid = tokens.front();
    invalid.attributes.risk = 2.0;
    expect(store.upsert(invalid) == UpsertCode::RejectedInvalid, "reject out-of-range attribute");

    expect(store.tick(3'000'000'000) == 5, "ttl expiry marks stale");
    expect(store.valid_tokens().empty(), "no valid tokens after ttl");
    expect(store.tick(9'000'000'000) == 5, "grace expiry evicts");
    expect(store.find(1)->status == TokenStatus::Evicted, "evicted status");
}

void test_session_invalidation() {
    RuleBasedTokenEncoder encoder;
    TokenStore store;
    for (const auto& t : encoder.encode(make_observation(0))) {
        store.upsert(t);
    }
    expect(store.invalidate_session(1, 1) == 5, "localization reset evicts session tokens");
    expect(store.valid_tokens().empty(), "nothing valid after session reset");
}

void test_esn_memory() {
    EsnConfig cfg;
    cfg.input_dim = kBehaviorInputDim;
    cfg.reservoir_dim = 32;
    EchoStateMemory a(cfg);
    EchoStateMemory b(cfg);
    const std::vector<double> u(kBehaviorInputDim, 0.5);
    for (int i = 0; i < 10; ++i) {
        a.update(u);
        b.update(u);
    }
    expect(a.state() == b.state(), "deterministic reservoir for same seed");
    expect(a.warmed_up(), "warmed up after washout");
    bool bounded = true;
    for (double x : a.state()) {
        bounded = bounded && std::isfinite(x) && std::abs(x) <= 1.0;
    }
    expect(bounded, "state bounded by tanh");

    const auto snap = a.snapshot();
    a.reset(MemoryResetReason::FloorChange);
    expect(!a.warmed_up() && a.epoch() == 1, "reset starts new epoch with washout");
    expect(a.restore(snap) && a.state() == b.state(), "restore snapshot");
}

void test_edge_policy() {
    RuleBasedTokenEncoder encoder;
    const auto tokens = encoder.encode(make_observation(0));
    EsnConfig cfg;
    cfg.input_dim = kBehaviorInputDim;
    cfg.reservoir_dim = 16;
    cfg.washout_steps = 2;
    EchoStateMemory memory(cfg);

    EdgePolicyConfig pcfg;
    pcfg.min_validation_samples = 10;
    EdgePolicyManager manager(pcfg, cfg.reservoir_dim + kBehaviorInputDim);

    auto d = manager.decide(tokens, memory);
    expect(d.fallback && d.action == BehaviorAction::Explore && d.target_token == 1001,
           "no active policy -> rule fallback explores best frontier");

    for (int i = 0; i < 30; ++i) {
        const auto u = make_behavior_input(tokens, {});
        memory.update(u);
        PolicyDecision explore{BehaviorAction::Explore, 1001, manager.active_version(), false, "shadow"};
        manager.report({explore, true, false, ""}, {memory.state(), u});
    }
    expect(manager.try_promote(), "validated candidate promoted");
    expect(manager.active_version() > 0, "active policy version set");

    d = manager.decide(tokens, memory);
    expect(!d.fallback && d.action == BehaviorAction::Explore, "active policy learned to explore");

    const auto u = make_behavior_input(tokens, {});
    for (int i = 0; i < 5; ++i) {
        PolicyDecision bad{BehaviorAction::Explore, 1001, manager.active_version(), false, "active"};
        manager.report({bad, false, true, "unreachable"}, {memory.state(), u});
    }
    expect(manager.active_version() == 0, "failure streak rolls back to rule fallback");
    expect(manager.decide(tokens, memory).fallback, "fallback after rollback");
}

}  // namespace

int main() {
    test_encoder_and_lifecycle();
    test_session_invalidation();
    test_esn_memory();
    test_edge_policy();
    if (failures > 0) {
        return EXIT_FAILURE;
    }
    std::cout << "cognition tests passed\n";
    return EXIT_SUCCESS;
}
