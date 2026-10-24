#include "cognition/semantic_token.h"

#include <algorithm>
#include <cmath>

namespace fork_spatial::cognition {
namespace {

bool finite_unit(double v) { return std::isfinite(v) && v >= 0.0 && v <= 1.0; }

bool attributes_valid(const SemanticAttributes& a) {
    return finite_unit(a.traversability) && finite_unit(a.goal_relevance) &&
           finite_unit(a.information_gain) && finite_unit(a.risk) && finite_unit(a.priority);
}

double clamp01(double v) { return std::clamp(v, 0.0, 1.0); }

constexpr std::uint64_t kSceneTokenId = 1;
constexpr std::uint64_t kGoalTokenId = 2;
constexpr std::uint64_t kRiskTokenId = 3;
constexpr std::uint64_t kFrontierTokenBase = 1'000;

}  // namespace

TokenStore::TokenStore(TokenStoreConfig config) : config_(config) {}

UpsertCode TokenStore::upsert(SemanticToken token) {
    if (!attributes_valid(token.attributes) || !finite_unit(token.confidence) ||
        token.confidence < config_.min_confidence || token.ttl_ns <= 0 ||
        token.resource.frame_id.empty()) {
        return UpsertCode::RejectedInvalid;
    }
    auto it = tokens_.find(token.token_id);
    if (it != tokens_.end() && it->second.status != TokenStatus::Evicted &&
        token.resource.revision < it->second.resource.revision) {
        return UpsertCode::RejectedStaleRevision;
    }
    const bool inserted = it == tokens_.end();
    token.status = TokenStatus::Valid;
    token.version = inserted ? 1 : it->second.version + 1;
    stale_since_.erase(token.token_id);
    tokens_[token.token_id] = std::move(token);
    ++store_version_;
    return inserted ? UpsertCode::Inserted : UpsertCode::Updated;
}

std::size_t TokenStore::tick(Timestamp now_ns) {
    std::size_t changed = 0;
    for (auto& [id, token] : tokens_) {
        if (token.status == TokenStatus::Valid && now_ns - token.stamp_ns > token.ttl_ns) {
            token.status = TokenStatus::Stale;
            stale_since_[id] = now_ns;
            ++changed;
        } else if (token.status == TokenStatus::Stale &&
                   now_ns - stale_since_[id] > config_.eviction_grace_ns) {
            token.status = TokenStatus::Evicted;
            stale_since_.erase(id);
            ++changed;
        }
    }
    if (changed > 0) {
        ++store_version_;
    }
    return changed;
}

std::size_t TokenStore::invalidate_session(std::uint32_t robot_id, std::uint32_t session_id) {
    std::size_t changed = 0;
    for (auto& [id, token] : tokens_) {
        if (token.status != TokenStatus::Evicted && token.resource.robot_id == robot_id &&
            token.resource.session_id == session_id) {
            token.status = TokenStatus::Evicted;
            stale_since_.erase(id);
            ++changed;
        }
    }
    if (changed > 0) {
        ++store_version_;
    }
    return changed;
}

std::size_t TokenStore::invalidate_frame(const std::string& frame_id) {
    std::size_t changed = 0;
    for (auto& [id, token] : tokens_) {
        if (token.status != TokenStatus::Evicted && token.resource.frame_id == frame_id) {
            token.status = TokenStatus::Evicted;
            stale_since_.erase(id);
            ++changed;
        }
    }
    if (changed > 0) {
        ++store_version_;
    }
    return changed;
}

std::optional<SemanticToken> TokenStore::find(std::uint64_t token_id) const {
    auto it = tokens_.find(token_id);
    if (it == tokens_.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::vector<SemanticToken> TokenStore::valid_tokens() const {
    std::vector<SemanticToken> out;
    for (const auto& [id, token] : tokens_) {
        if (token.status == TokenStatus::Valid) {
            out.push_back(token);
        }
    }
    std::sort(out.begin(), out.end(),
              [](const auto& a, const auto& b) { return a.token_id < b.token_id; });
    return out;
}

RuleBasedTokenEncoder::RuleBasedTokenEncoder(RuleEncoderConfig config) : config_(config) {}

std::vector<SemanticToken> RuleBasedTokenEncoder::encode(const GeometricObservation& obs) {
    std::vector<SemanticToken> out;
    const double loc_conf = obs.localization_valid ? clamp01(obs.localization_quality) : 0.0;
    auto make = [&](std::uint64_t id, TokenKind kind) {
        SemanticToken t;
        t.token_id = id;
        t.kind = kind;
        t.resource = obs.map;
        t.stamp_ns = obs.stamp_ns;
        t.ttl_ns = config_.token_ttl_ns;
        t.confidence = loc_conf;
        return t;
    };

    auto scene = make(kSceneTokenId, TokenKind::Scene);
    scene.attributes.traversability = clamp01(1.0 - obs.occupancy_ratio);
    scene.attributes.risk = clamp01(obs.occupancy_ratio);
    scene.attributes.priority = 0.5;
    out.push_back(scene);

    auto risk = make(kRiskTokenId, TokenKind::Risk);
    risk.attributes.risk = clamp01((1.0 - loc_conf) * 0.5 + 0.15 * obs.recent_failures);
    risk.attributes.priority = risk.attributes.risk;
    out.push_back(risk);

    double best_gain = 0.0;
    for (const auto& f : obs.frontiers) {
        best_gain = std::max(best_gain, f.information_gain);
    }

    for (const auto& f : obs.frontiers) {
        auto tok = make(kFrontierTokenBase + f.id, TokenKind::Frontier);
        const double dist = std::hypot(f.x - obs.robot_x, f.y - obs.robot_y, f.z - obs.robot_z);
        const double proximity = std::exp(-dist / config_.distance_scale_m);
        tok.anchor = {f.x, f.y, f.z};
        tok.attributes.information_gain = best_gain > 0.0 ? clamp01(f.information_gain / best_gain) : 0.0;
        tok.attributes.traversability = clamp01(f.traversability);
        tok.attributes.priority = clamp01(0.6 * tok.attributes.information_gain + 0.4 * proximity);
        out.push_back(tok);
    }

    if (obs.goal && obs.goal->size() >= 3) {
        auto goal = make(kGoalTokenId, TokenKind::Goal);
        const auto& g = *obs.goal;
        const double dist = std::hypot(g[0] - obs.robot_x, g[1] - obs.robot_y, g[2] - obs.robot_z);
        goal.anchor = g;
        goal.attributes.goal_relevance = 1.0;
        goal.attributes.priority = clamp01(0.5 + 0.5 * std::exp(-dist / config_.distance_scale_m));
        out.push_back(goal);
    }
    return out;
}

}  // namespace fork_spatial::cognition
