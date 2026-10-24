#ifndef FORK_SPATIAL_CONTRACT_SPATIAL_STATE_H
#define FORK_SPATIAL_CONTRACT_SPATIAL_STATE_H

#include <cstdint>
#include <string>
#include <string_view>

namespace fork_spatial::contract {

// 空间状态契约：三个仓库（MotionSLAM_ws / legged-navigation-runtime / 本仓库）
// 共用的"身份·版本·时间·可信度·原因码"。只定义字段与规则，不定义传输。
// 各仓库现有字段到本契约的映射见 docs/contracts/spatial_state.md。

// ---------- 1. 身份与版本链 ----------
// revision = 资产内容版本（MotionSLAM map_version / layer_version）。
// generation = 有效期代数（MotionSLAM context_generation / SHM generation）；
//              generation 变化意味着旧状态、旧子目标、旧轨迹全部作废。
// frame_id 必填、无默认值：Go2 默认 world、规划器默认 map，不能隐式假定。
struct AssetRef {
    std::uint32_t robot_id{};
    std::uint32_t session_id{};
    std::string asset_id;          // 建议编码 building/floor/map/tile
    std::uint64_t revision{};
    std::uint64_t generation{};
    std::string frame_id;
    std::string asset_type;        // occupancy / octvox / tile / pose_graph …（可选）
    std::uint64_t content_hash{};  // 已提交内容的校验，0 = 未提供

    bool operator==(const AssetRef&) const = default;
};

// 同一 asset_id 下，生命周期上 b 是否仍可消费为 a 的同代状态。
[[nodiscard]] bool same_generation(const AssetRef& a, const AssetRef& b) noexcept;
[[nodiscard]] bool validate(const AssetRef& ref, std::string& error);

// ---------- 1b. 增量同步 ----------
// 身份(asset_id) / 版本(revision) / 传输编码(codec) 三者严格分开：Token 流只是编码，
// 不承担资产身份。增量必须声明基线版本；消费方基线不符不得盲目应用，先补齐或要快照。
struct DeltaRef {
    std::string asset_id;
    std::uint64_t base_revision{};
    std::uint64_t target_revision{};
    std::uint64_t generation{};
    std::string codec_id;      // raw_diff / quantized / token_stream …
    std::uint64_t checksum{};  // 解码后校验，0 = 未提供
};

enum class DeltaDisposition : std::uint8_t {
    Apply,            // base == 本地 revision
    AlreadyApplied,   // target <= 本地 revision
    MissingDeltas,    // 本地落后于 base 或与区间交叠 → 补齐缺失增量或请求快照
    StaleGeneration,  // asset / generation 不匹配 → 丢弃并重同步
};
[[nodiscard]] DeltaDisposition dispose(const DeltaRef& delta, const AssetRef& local) noexcept;

// ---------- 2. 时间 ----------
enum class ClockDomain : std::uint8_t {
    Monotonic,   // 端侧单机节拍（steady_clock / ROS steady）
    WallUtc,     // 跨机/跨进程对时后的 UTC
};

struct Stamp {
    std::int64_t stamp_ns{};
    std::int64_t valid_until_ns{};   // 0 = 不设有效期
    ClockDomain clock{ClockDomain::Monotonic};

    [[nodiscard]] bool expired_at(std::int64_t now_ns) const noexcept {
        return valid_until_ns != 0 && now_ns > valid_until_ns;
    }
};

// ---------- 3. 原因码（唯一一套，替代 mission::FailureCode 与 failure_code 字符串） ----------
enum class ReasonCode : std::uint8_t {
    None,
    NoWaypoints,
    PlanFailed,            // 单次规划失败
    PlanFailedRepeated,    // 连续规划失败超阈值
    ReplanTimeout,         // 规划器在期限内未产出
    Stuck,                 // 位姿无进展
    LocalizationLost,      // 定位失效
    LocalizationJump,      // 漂移 / 跳变超阈值
    RelocalizationTimeout, // 重定位搜索或配准超时
    FloorTransition,       // 楼层 / 地图切换中
    AssetStale,            // generation 不匹配，旧状态被拒
    LinkLost,              // 断链（仅上报；端侧继续本地自主）
    SafetyVeto,            // 度量层否决（含类别无关的障碍、原玻璃分支）
    ResyncRequired,        // 增量基线缺失 / 校验失败，需补齐或快照
    Timeout,               // 任务级超时
    Canceled,
};
[[nodiscard]] std::string_view name(ReasonCode code) noexcept;

// ---------- 4. 空间状态可信度 ----------
// 合并：MotionSLAM 规划§6 的 NORMAL_3D/DEGRADED_2D/RELOCALIZING/VERIFYING/HOLD、
// task_runtime::SafetyMode、legged-navigation RecoveryState、Go2 /lio/* 话题。
enum class LocalizationMode : std::uint8_t {
    Normal3D,
    Degraded2D,
    Relocalizing,
    Verifying,
    Hold,
};
[[nodiscard]] std::string_view name(LocalizationMode mode) noexcept;

// 可信度决定 Runtime 还给执行层多少授权；这是"可信度驱动任务恢复"的核心规则。
enum class ExecutionAuthority : std::uint8_t {
    Full,        // Normal3D：正常派发
    Restricted,  // Degraded2D：限速 / 限域，仍可推进
    Suspended,   // Relocalizing / Verifying：StopTrajectory，任务保留，等恢复
    Revoked,     // Hold：收回授权，任务进 Holding，需显式恢复
};
[[nodiscard]] ExecutionAuthority authority_for(LocalizationMode mode) noexcept;

struct LocalizationHealth {
    LocalizationMode mode{LocalizationMode::Normal3D};
    double quality{1.0};            // [0,1]，喂 Token.confidence / localization_quality
    double pose_jump_m{0.0};        // ← LNR pose_jump_m / Go2 drift_estimate
    double alignment_fitness{0.0};  // ← LNR alignment_fitness / Go2 /lio/relocation/fitness
    std::uint32_t healthy_frames{}; // ← LNR healthy_frames_to_resume 计数
    bool stance_ok{true};           // ← Go2 /local_map/stance_ok
    ReasonCode reason{ReasonCode::None};

    [[nodiscard]] ExecutionAuthority authority() const noexcept { return authority_for(mode); }
    [[nodiscard]] bool usable() const noexcept {
        return mode == LocalizationMode::Normal3D || mode == LocalizationMode::Degraded2D;
    }
};

// 一条带可信度和版本的空间状态观测——所有上行消息的最小公共头。
struct SpatialStateHeader {
    AssetRef asset;
    Stamp stamp;
    LocalizationHealth health;
};

// 消费方在使用前调用：代数不匹配 → AssetStale；过期 → 拒收；mode 不可用 → 拒收。
[[nodiscard]] ReasonCode admit(const SpatialStateHeader& header,
                               const AssetRef& expected,
                               std::int64_t now_ns) noexcept;

}  // namespace fork_spatial::contract

#endif
