# A1 → cuRobo → A2：30° 容忍实验

## 本轮目标

按用户要求，Anchor 物体姿态验收改为位置误差 ≤5 cm、旋转误差 ≤30°。
两项分别检查；优化奖励保留原来的加权误差。Replay、MPC、PPO、core
交接共用同一实现，参数写进 artifact 和 PPO 报告，没有样本名称分支。
抓持保持和真实 canonical 边界相容检查仍分别报告。

仍用已验证的 oracle 时间表：A1 core 为帧 35–76；中间 RELAXED 为帧
76–82，即 0.2 秒；A2 core 为帧 82–101。中间规划须从 A1 实测出口到
A2 实测入口，并保留抓持、携带物体碰撞及原时间预算。

作业流程支持复用已有局部输入、冻结的 cuRobo predecessor，以及通过某个
Anchor 就结束的连续验证。A2 的开发入口由真实前缀执行获得；不能从 GT
构造空中物体或手的位置。A1 尚未合格时，作业不会把 A2/连接标成通过。

## 已完成的实验

| 实验 | 改动 | 实测结果 |
|---|---|---|
| r51 / 648534 | 旋转尺度 30°，仍用组合误差验收 | A1 未通过；最佳 PPO 在帧 53 失败，耗时 282.37 秒 |
| r52 / 648538 | 位置 ≤5 cm 且旋转 ≤30° 独立验收 | 首个 40 帧预测窗通过，真实 commit 推进到帧 49；第二窗最佳 PPO 在帧 72 失败，A1 未通过；耗时 414.41 秒 |

r51 暴露了组合判定的含义：2.4 cm / 29° 分别在容忍范围内，仍可能被
加权范数拒绝。因此 r52 起把验收与优化评分分开。这是用户要求的实验
容忍度修改，不是证明旧数据在新标准下自动完成整个交互。

r52 还暴露两处可复用问题，已修复：

1. PPO 训练中曾有完整通过预测窗的 sampled episode，但代码只保留确定性
   均值策略，丢掉了已找到的命令。现在保存并返回成功 sampled commands；
   它们仍须经过 commit、完整 padded Replay 和 exported-core Replay。
   这表示命令候选可用，不表示策略均值已收敛。
2. 已抓持状态的手指会受力偏离 drive target。旧预检查把这份已有预紧当作
   新命令跳变，第二窗 baseline 的 nominal hand velocity ratio 达到 6.305。
   现在从保存的驱动目标检查命令速度，并在抓持 snapshot 恢复、残差投影
   和下一 Anchor 的 padded 入口保留它；物理 q/dq 仍取实测值。

针对该实际故障的现有局部检查及一项预紧回归共 13 项通过，约 0.5 秒。
未增加跨样本测试、实测加速度门槛或新的 torque 门槛。

## 后续物理实验

| 实验 | 结果 | 暴露的实际问题 |
|---|---|---|
| r53 / 648548 | 局部搜索找到命令，但完整命令预检查失败 | 窗口拼接处手指目标跳变 0.354 rad，速度比例 3.029；尚无有效 A1 |
| r54 / 648563 | 137.85 秒，失败于帧 69 | 完整预测候选通过后，重复执行其前缀出现 core 物体误差；尚无有效 A1 |
| r55 / 648645 | 129.72 秒，失败于帧 78 | MPPI 已保存实际到达的前缀；PPO 两轮、6 episodes 找到完整通过的末段候选，重复执行时 post-context 失去抓持 |
| r56 / 648648 | 151.33 秒，完整 padded Replay 失败于帧 73 | 分窗预测均可通过，但整段无重置运行首次超差：位置 4.868 cm，旋转 30.104°；当时仍有对向抓持 |
| r57 / 648666 | 44.40 秒；padded 全段通过，snapshot 恢复后的 core 在帧 73 失败 | padded core 最大位置误差 4.698 cm、旋转 28.225°，帧 76–82 持续 grasped；尚无有效 A1 artifact |

r53 的修复保留所有已有 drive target，并将局部提案的手指目标按 URDF
速度投影；不能因为接触标签暂时为 free 就丢弃预紧。r54/r55 显示成功
预测与重新执行并不等价；目前不能把原因唯一归结为 PhysX 内部缓存。

现在 MPPI 与 PPO 都保存成功物理预测中实际到达的前缀 snapshot 和实测
trace，用于后续局部开发。它们不是最终验收：拼接后的完整 padded Replay、
exported-core Replay 仍须独立通过；最终前缀执行不允许任何 segment reset。
对应代码为 `386f18b` / `68d5eae`。

同时修复一个真实连接接口缺口：free RELAXED 原来冻结手指，不能到达
已合成 Anchor 的不同 pregrasp 手形。`8a50ef0` 增加原时间预算内的手指
边界插值，检查关节范围/速度及逐物理步实测碰撞；cuRobo 仍负责手腕路径。
规划几何仍取入口手形，变化后的几何由实测检查；尚未宣称整段已验证。
grasped RELAXED 继续保留已有握紧目标，不用 planner 改变抓持模式。

所有改动无 sample-name 分支；只运行相关的 14 个已有/故障回归检查，
0.63 秒。尚未用第二个 sequence 验证 transfer。

## 当前验证

r58 / 648683：interactive/H200，申请 20 分钟。完整 A1 padded/core 通过后
才会取得 A2 的真实开发入口，并验证 A1 → 0.2 秒 cuRobo → A2。

`4a863ee` 针对已发生的“分窗成功、整段失败”增加局部整段 PPO 修正：
从原先真实取得的 synth_start 状态开始，每个 episode 执行整段 padded
Anchor。成功候选须通过 fresh padded Replay 才能被选择；若重放失败，
训练继续。通过后仍执行原有 exported-core Replay。没有改变容忍度、
控制器、source 时间或最终无 reset 的条件。

这修复了优化流程不能使用整段物理失败反馈的问题；尚未证明它能让 A1
通过。r56 的入口段在帧 49 已出现 0.473 mm 手腕位置差、0.00137 rad
旋转差和约 0.0042 rad hand-q 最大偏差，帧 69 差异进一步扩大；没有证据足以把
所有差异唯一归因于某个 PhysX 缓存或 solver bug。

r57 的 warm-start padded Replay 直接通过，未运行 whole-anchor PPO。
`f02be5d` 将 exported-core 开发验收改为：恢复真实 synth_start 开发
snapshot → 实际执行 padding → 直接执行 core。core 边界不恢复、不设置
机器人/物体状态，入口 contact 仍须符合 oracle。该路径满足“exported core
从 physically reached core entry 执行”；最终 initial → cuRobo → A1
是否能到达同一 canonical entry，仍由独立连续前缀检查决定。
这是实际入口执行的机制修正，没有改变 30° 或 contact 判定；r58 尚待验证。

r55 实际状态视频：
`outputs/oracle_floating_a1_a2_prefixstate_r55/anchor_0/replay/floating_xhand_actual.mp4`。
47 帧 / 30 fps，1.5667 秒，来自实测 PhysX mesh transforms；这是失败的
局部 Replay，不能作为成功 Anchor 或完整 4.8 秒轨迹。

更新的 r56 整段尝试视频：
`outputs/oracle_floating_a1_a2_ppoprefix_r56/anchor_0/refined_replay/floating_xhand_actual.mp4`。
44 帧 / 30 fps，1.4667 秒，止于首次超差。A1 未通过，A2 与中间 bridge
尚未执行；不能把缺少有效 Anchor 边界称为 cuRobo 连接失败。

首个通过的 padded 实际状态视频：
`outputs/oracle_floating_a1_a2_wholeppo_r57/anchor_0/replay/floating_xhand_actual.mp4`。
53 帧 / 30 fps，1.7667 秒，对应源帧 29–82。只证明这次 A1 padded
执行有效；exported-core / canonical handoff / A2 尚未通过。
