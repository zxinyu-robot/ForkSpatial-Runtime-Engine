# Fork Spatial Runtime

[![CI](https://github.com/zxinyu-robot/fork-spatial-runtime/actions/workflows/ci.yml/badge.svg)](https://github.com/zxinyu-robot/fork-spatial-runtime/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C.svg)](https://isocpp.org/)

面向弱网 / 断网自主机器人的**语义状态管理、时序行为记忆与端侧持续学习调度架构**。

> A semantic-state, behavior-memory and edge continual-learning runtime for autonomous robots under weak or no connectivity.

三个核心模块分别解决状态表征、历史记忆和在线决策问题，它们是同一个闭环里的三个环节，不是彼此独立的算法：

| 模块 | 职责 | 关键产物 |
|---|---|---|
| Token 编码与资源管理 | 几何观测 → 语义价值 Token，管理来源、版本、时效、置信度与淘汰 | 带语义属性和资源引用的 Token |
| ESN 行为时序记忆 | 融合连续场景 Token、历史行为与执行反馈 | 动态记忆状态 \(x_t\) |
| 端侧在线学习与决策 | 本地推理、策略选择、任务调度；候选策略异步学习，验证后发布、可回滚 | 行为决策、策略版本 |

```text
World State（LIO / GridMap / Frontier / 位姿 / 执行反馈）
      │ GeometricObservation                    ┌──────────────────────────┐
      ▼                                         │ 状态内核 SpatialRuntime   │
Token Encoder + TokenStore ── 资源生命周期 ─────┤ 状态生命周期 / 原子事务   │
      │ SemanticToken                           │ / 固定延迟窗口            │
      ▼                                         └──────────────────────────┘
EchoStateMemory（x_t）
      │
      ▼
EdgePolicyManager ── active 实时决策 │ candidate 在线学习 → 验证 → promote / rollback
      │ PolicyDecision
      ▼
机器人任务执行层（BT-Planner / Task Runtime）+ 度量安全监督（可达性 / 碰撞 / 安全降级，可否决）
      │ DecisionOutcome（成功 / 否决 / failure_code）
      └──────────────▶ 回流 Token、ESN 与在线学习
```

两条边界：**语义价值 ≠ 几何安全许可**（可行性由规划与度量安全监督否决）；**ESN 动态记忆 ≠ 长期经验库**（长期经验与策略更新归端侧学习模块）。模型推理、调度和学习都在端侧闭环，云端只负责预训练、数据分析和策略分发。

契约文档：[SpatialState](docs/contracts/spatial_state.md) · [SemanticToken](docs/contracts/semantic_token.md) · [BehaviorMemory](docs/contracts/behavior_memory.md) · [EdgePolicy](docs/contracts/edge_policy.md) · [MissionOrchestration](docs/contracts/mission_orchestration.md) · [状态内核架构](docs/architecture.md)

## 机器人适配

架构与机型无关，各机器人通过适配层接入：

- 四足 Unitree Go2：[MotionSLAM_ws](https://github.com/zxinyu-robot/MotionSLAM_ws)（跨楼层自主导航 + 恢复机制）。BT 运行时是执行中枢，插件只提供证据或建议，不直接拿执行权；本仓库的空间状态契约通过契约桥接入 BT 运行时（`ContractGate` + 故障演练，[MotionSLAM_ws #12](https://github.com/zxinyu-robot/MotionSLAM_ws/pull/12) 审核中）。
- 人形 / 异构恢复插件：[Legged Navigation Runtime](https://github.com/zxinyu-robot/legged-navigation-runtime)（定位故障恢复、多楼层导航）。其重定位状态机以恢复插件形式挂到 MotionSLAM BT 运行时，四足先闭环。
- 无人机：后续接入
- 多机弱网协同（DDS）：后续按需接 [Swarm Spatial Link](https://github.com/zxinyu-robot/swarm-spatial-link)

## 当前已实现

认知层（`src/cognition/`）：

- `RuleBasedTokenEncoder` + `TokenStore`：规则化语义编码；资源生命周期 Valid → Stale → Evicted；拒收 revision 回退；会话 / 坐标系整体作废。
- `EchoStateMemory`：确定性稀疏储备池，按谱半径缩放、带泄漏积分；重置规则包括任务切换、定位重置、跨楼层；支持 washout 和快照恢复。
- `EdgePolicyManager`：active / candidate 双策略；旁路比对门槛（样本数、否决率、成功率）；连续失败回滚；规则兜底。

空间状态契约（`src/contract/`，三个仓库共用，只定义字段与规则，不定义传输）：

- 身份与版本链：`AssetRef`（revision / generation / localization_epoch / frame_id）、`DeltaRef` 与 `dispose()`；generation 变化时旧状态全部作废，localization_epoch 变化时只作废与位姿绑定的状态。
- 时间与可信度：`Stamp`（时钟域、有效期）、`LocalizationHealth`（定位模式、质量、位姿跳变、重定位进度 `candidate_found` / `alignment_complete` / `alignment_success`）、`ReasonCode`；只有 Normal3D / Degraded2D 允许执行。
- 任务执行契约：`ExecutionLease`（owner / epoch / expiry / start_deadline）、`IntentEnvelope`、幂等判定；`admit()` 对状态给出 Accept / Reverify / Hold / Reject，`dispose()` 对租约给出接受或具体失效原因。
- 故障注入 v1（`fault_injection.*`）：定位丢失、位姿跳变、换图、定位重置、过期、断链、租约过期、过期执行者、超时等确定性场景，逐步给出 `admit()` 与 `dispose()` 判决。

任务编排（`src/mission/`）：`MissionOrchestrator` 是机器人无关的参考执行器 / 测试夹具，负责子目标派发、到达判定、恢复阶梯和终态，不做几何规划与设备控制。

状态内核（`src/runtime/`）：

- 状态键：机器人、会话、状态类型、时间戳和实例编号。
- 状态生命周期：Requested → Initialized → Active / Fixed → Marginalized → Archived。
- 因子批次校验：重复 ID、缺失状态、非法协方差和迟到观测。
- 原子提交语义：后端拒绝事务时，不提交候选状态和图版本。
- 固定延迟窗口策略，以及由后端生成边缘化先验的接口。
- 后端无关的 `IOptimizerBackend`、`IFactorProvider` 和 `ISensorAdapter` 接口。
- Linux/macOS CI 和无需第三方依赖的单元测试。

## 尚未实现

- GTSAM、Ceres 或 g2o 的生产后端。
- ROS 2/DDS 适配器、共享内存传输和持久化。
- 学习型 Token 编码器、ESN 读出层的离线预训练管线、策略持久化与分发。
- 多机器人子图融合、回环检测和可视化界面。
- 契约生成的 DDS IDL、弱网传输调度与 Spatial Delta 协议。
- 实时性能 benchmark 与真机数据集验证（Go2 回放基线进行中）。

这些内容属于后续路线，不是当前版本的完成能力。

## 快速开始

要求：CMake 3.25+（`CMakePresets.json` 为 v6）、Make 和支持 C++20 的编译器。CMake 3.20–3.24 可以不用 preset：`cmake -S . -B build/default -DBUILD_TESTING=ON && cmake --build build/default && (cd build/default && ctest --output-on-failure)`。

```bash
cmake --preset default
cmake --build --preset default
ctest --preset default --output-on-failure
./build/default/examples/cpp/runtime_demo
```

预期演示输出：

```text
accepted graph version: 1
active states: 1
backend operations: 3
```

## 最小用法

```cpp
using namespace fork_spatial::runtime;

SpatialRuntime runtime(
    std::make_unique<MyOptimizerBackend>(),
    std::make_unique<FixedLagWindowPolicy>(2'000'000'000));

FactorBatch batch;
batch.state_requests.push_back({pose_key, initial_pose});

const SubmitResult result = runtime.submit(batch);
if (result) {
    const EstimateSnapshot estimate = runtime.optimize();
}
```

优化后端必须保证 `apply()` 的原子性。边缘化分为“计算先验”和“提交事务”两步，避免后端部分修改后运行时状态无法回滚。

## 项目结构

```text
src/runtime/
  model.h             # 状态、因子和事务数据模型
  interfaces.h        # 传感器、因子提供器、窗口和后端接口
  state_store.*       # 状态生命周期与索引
  spatial_runtime.*   # 批次校验、提交、边缘化和优化入口
src/cognition/
  semantic_token.*    # Token 编码与资源生命周期
  behavior_memory.*   # ESN 行为时序记忆
  edge_policy.*       # 端侧决策、在线学习与版本管理
src/mission/
  mission_orchestrator.*  # 参考执行器 / 测试夹具（子目标推进/到达判定/恢复阶梯）
src/contract/
  spatial_state.*         # 跨仓库空间状态契约：AssetRef / Stamp / LocalizationHealth / ReasonCode / ExecutionLease / admit()
  fault_injection.*       # 故障注入 v1：确定性失效场景 → admit()/dispose() 判决
examples/cpp/
  runtime_demo.cpp
  test_spatial_runtime.cpp
  test_cognition.cpp
  test_mission.cpp
  test_contract.cpp
docs/
  architecture.md
  contracts/          # SpatialState / SemanticToken / BehaviorMemory / EdgePolicy / MissionOrchestration 契约
  plan/               # 完整技术路线、项目执行计划、评价体系等规划文档
```

## 路线图

阶段与进度以 [项目执行计划](docs/plan/项目执行计划.md) 为准（总览见 [docs/plan](docs/plan/README.md)）。关键路径：

**A 空间状态契约 → A2 任务执行契约 → B 插件化基座 → C Go2 旁路接入 →（D 反馈回流 ∥ D' 断网自治）→ E 电梯用例 + 受限介入 → F 传输调度 + 协议 + 压测**，之后才是 G 异构 + 世界模型、H 多机。

当前：A、A2 已完成；B 剩故障演练接入 MotionSLAM（审核中）和 Go2 回放基线。

状态内核的设计边界见 [架构说明](docs/architecture.md)。

## License

[MIT](LICENSE)
