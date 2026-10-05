# A2：源帧 82 边界与物理入口

> 本页记录截至 r65 的历史结果。2026-10-06 的 r68 已通过 A2 padded
> 和 exported-core 验收，见 [原因定位与成功结果](A2_GRASP_DIAGNOSIS_20261006.md)。

## 目标定义

用户已明确：A2 的目标入口是 **TACO 源轨迹第 82 帧的边界目标**。
A1 的实际 core 出口在帧 76；两者不能视为同一状态。

流程为：A1 实测出口 76 → cuRobo RELAXED（0.2 秒）→ A2 的物理
canonical 入口 82 → A2 core 82–101。最终 rollout 不允许状态赋值、
物体瞬移或延长 source 时间。

局部开发使用 r58 成功 A1 padded rollout 在帧 76 的 PhysX snapshot，
作为 A2 synthesis 76–107 的上下文起点。它有实际 trace 和来源校验，
明确标为 `local_synthesis_context_only`，**不是 A2 canonical 入口**。
A2 必须实际执行到帧 82，再检查自身源边界目标及 grasped 接触状态。
合格后保存的是实测入口状态，不是 TACO pose 直接构造的空中状态。

## 已完成实验

| 实验 | 物理结果 | 结论 |
|---|---|---|
| r60 / 648701 | 120.71 秒。Replay 帧 82：位置误差 4.00 cm，旋转 35.82°；不满足 30°。MPPI/PPO 候选能越过入口，但未完成 core，PPO zero candidate 帧 87 失去抓持 | A2 尚未有效；真实难点包括入口对齐与 core 抓持保持 |
| r61 / 648705 | 100.83 秒。冻结所有手指 drive target：Replay 帧 82 旋转误差 47.72°。最佳搜索 seed 帧 80 已失去抓持；253 个 PPO episodes 无成功 | 全冻结没有改善，已撤回；padding 内全零奖励使失败候选无法得到有效区分 |
| r62 / 648713 | 146.60 秒，失败。PPO 在最后一轮到帧 84，因 core 物体偏差退出；没有有效 A2 artifact | 抓持反馈产生有效进展，但仍有 wrist/object 目标不相容 |
| r63 / 648730 | 166.19 秒，失败。nominal Replay 入口帧 82 通过：4.54 cm / 18.10°；首次 core 超差帧 83。PPO 最后一轮 deterministic 到帧 98，因 core 物体偏差退出 | measured-grasp wrist 提案解决了本轮入口失败，但 A2 全段尚未通过 |
| r64 / 648740 | 148.31 秒，失败。保留了实际通过源帧 76–96 的 20 帧前缀；第二窗口在帧 97 物体超差 | 分窗开发起点虽是真实 snapshot，但末段仍失败；不能把有效前缀叫作完整 Anchor |
| r65 / 648744 | 248.32 秒，失败。分窗失败后确实执行了完整连续 Replay 和整段 PPO；fresh Replay 到帧 91，入口 4.37 cm / 15.82° 通过。整段 PPO 60 updates / 253 episodes：最佳评分候选帧 95 原生对向接触丢失；最后 deterministic 到帧 97，因物体超差退出 | fallback 已真正接通；当前仍是连续 core 的实际抓持/跟踪失败，不是入口定义或程序提前退出 |

r59 作业因入口定义纠正而在排队时取消，没有物理实验结果。

## 可复用修复

1. canonical 入口检查从每个 Anchor 的 `core_start` 读取自身源边界。
   没有帧 82 或 sample-name 的运行时分支。报告同时保存目标、实测误差，
   不用 context snapshot 替代目标。
2. MPPI 计入失败边界的实测奖励；以前未计入，入口失败候选失去反馈。
3. 当时对 grasped → grasped 的 nominal retarget，根据实测手指与已保存驱动
   目标的差值识别预紧方向，允许进一步闭合或调整。MPPI/PPO 仍可探索
   有界残差。该规则是初始化提案，不能代替物理抓持验收。
4. padding 不追踪 human reference，但已有抓持的保持提供接触奖励。
   使用既有 thumb AND finger contact 奖励；没有放宽 core 容忍度或增加
   等待。r61 的全零 PPO feedback 是本次修改的直接证据。
5. r62 的 object-relative fallback 过渡长达 0.514 秒，晚于 0.2 秒的
   core 入口；帧 82 仍有 3.12 cm / 38.76° 偏差。持续抓持的 nominal
   wrist 现在由源物体轨迹和实测 hand-object transform 生成；入口位置/
   速度过渡只在已有 padding 内完成。原先假设 human retarget 握法与实际
   机器人握法一致，会产生相互冲突的 wrist/object 目标。新提案仍须通过
   完整 padded/core 物理验收；不改变 source 时间或 30° 容忍度。
6. r63 的局部 PPO 曾物理通过前 20 帧，未来第 22 帧预测失败，代码却
   丢弃全部前缀。现在允许将已通过每帧物理检查的 20 帧 actual prefix
   用于局部 MPC commit，从其真实状态继续优化末段。未知未来控制仅为
   提案，延续已有残差以免跳回原始 retarget。报告保留 `success=false`
   和 future failure；完整 padded/core Replay 仍是最终硬 gate。
   此修复仅开启局部 floating MPC 的前缀复用，整段 Replay acceptance
   callback 和 final continuous execution 不会接受失败前缀作为成功。
7. 分窗 refinement 失败时，原代码直接退出，整段 PPO 无法处理这种
   连续执行问题。现在保留已有 prefix / 最佳未来控制提案，先从原始
   物理 context 做完整连续 Replay，再必要时进入整段 PPO。失败提案
   标为 `proposal_only`，物理验收和最终 source 时间约束不变。

第 3 项的 drive-error 符号推断在 r66 暴露错误，现已由 XHand 闭合
方向元数据替代，并统一到 MPC/PPO 残差投影，详见上述新报告。

相关 16 个已有/故障回归检查通过（0.59 秒）；没有展开跨样本测试。
代码结构可复用，第二个 sequence 的 transfer 尚未验证。

固定验收条件没有改变：物体参考 5 cm / 30°；真实抓持保持与 boundary
compatibility 单独检查。r65 fresh Replay 帧 91 的对向接触仍在，但
hand-object 相对旋转 0.357 rad 超过已有保持阈值 0.35 rad；整段 PPO
最佳评分候选帧 95 相对变换偏差小，却已没有原生对向接触。两种失败
必须区分，不能把“仍有接触”与“整个 Anchor 有效”混为一谈。
源轨迹 retarget 的 A2 core 手物相对旋转最大变化约 5.51°；目前没有
证据支持通过放宽保持阈值来解决这些连续物理失败。

修复的实测收益：nominal 入口旋转误差从 r62 的 47.35° 降到 r63 的
18.10°；PPO 候选能到帧 98，但这不是完整 A2 成功。r64 验证了前缀
复用分支，r65 验证了失败分窗后的整段 fallback。完整局部 artifact
仍需 padded 和 exported-core 两项都成功，本轮没有绕过这些条件。

## 当前边界

A1 已有局部有效 padded/core artifact（r58）；A2 尚无有效 artifact。
A1 → A2 的 0.2 秒 cuRobo 连接尚未执行，因此不能宣称连接可行或不可行。
完整初始前缀仍在首段 cuRobo canonical 规划碰撞检查处被拒绝；完整
4.8 秒无 reset rollout 尚未完成。

最新 A2 实测视频（r65 fresh continuous Replay，源帧 76–91，15 帧，
30 fps，0.5 秒；失败的局部动作，不是完整 4.8 秒结果）：
`outputs/oracle_floating_a2_continuous_fallback_r65/anchor_1/refined_replay/floating_xhand_actual.mp4`。
视频仅渲染记录的 PhysX mesh transforms，不运行 GT 动画或额外模拟。

对照图：`docs/evidence/oracle_floating_20261005/a2_source_boundary/r62_r63_boundary_errors.png`。
机器报告：同目录 `r60_result.json` 至 `r65_result.json`（r59 为取消作业）。
