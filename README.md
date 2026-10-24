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

契约文档：[SemanticToken](docs/contracts/semantic_token.md) · [BehaviorMemory](docs/contracts/behavior_memory.md) · [EdgePolicy](docs/contracts/edge_policy.md) · [状态内核架构](docs/architecture.md)

## 机器人适配

架构与机型无关，各机器人通过适配层接入：

- 四足 Unitree Go2：[MotionSLAM_ws](https://github.com/zxinyu-robot/MotionSLAM_ws)（跨楼层自主探索导航 + 恢复机制）
- 人形：[Legged Navigation Runtime](https://github.com/zxinyu-robot/legged-navigation-runtime)（定位故障恢复、多楼层导航）
- 无人机：后续接入
- 多机弱网协同（DDS）：后续按需接 [Swarm Spatial Link](https://github.com/zxinyu-robot/swarm-spatial-link)

## 当前已实现

认知层（`src/cognition/`）：

- `RuleBasedTokenEncoder` + `TokenStore`：规则化语义编码；资源生命周期 Valid → Stale → Evicted；拒收 revision 回退；会话 / 坐标系整体作废。
- `EchoStateMemory`：确定性稀疏储备池，按谱半径缩放、带泄漏积分；重置规则包括任务切换、定位重置、跨楼层；支持 washout 和快照恢复。
- `EdgePolicyManager`：active / candidate 双策略；影子验证门槛（样本数、否决率、成功率）；连续失败回滚；规则兜底。

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
- 实时性能 benchmark 与真机数据集验证。

这些内容属于后续路线，不是当前版本的完成能力。

## 快速开始

要求：CMake 3.20+、Make 和支持 C++20 的编译器。

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
  mission_orchestrator.*  # 机器人无关任务编排（子目标推进/到达判定/恢复阶梯）
examples/cpp/
  runtime_demo.cpp
  test_spatial_runtime.cpp
  test_cognition.cpp
  test_mission.cpp
docs/
  architecture.md
  contracts/          # SemanticToken / BehaviorMemory / EdgePolicy / MissionOrchestration 契约
```

## 路线图

1. 增加 GTSAM iSAM2 适配器和 SE(3) 状态类型约束。
2. 增加乱序观测缓冲、并发读快照和可重复 benchmark。
3. 增加 ROS 2 消息适配器与 rosbag 回放示例。
4. 使用公开数据集报告吞吐、延迟和内存占用。

详细设计边界见 [架构说明](docs/architecture.md)。

## License

[MIT](LICENSE)
