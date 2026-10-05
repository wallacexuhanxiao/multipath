# A2：源帧 82 边界与物理入口

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

r59 作业因入口定义纠正而在排队时取消，没有物理实验结果。

## 可复用修复

1. canonical 入口检查从每个 Anchor 的 `core_start` 读取自身源边界。
   没有帧 82 或 sample-name 的运行时分支。报告同时保存目标、实测误差，
   不用 context snapshot 替代目标。
2. MPPI 计入失败边界的实测奖励；以前未计入，入口失败候选失去反馈。
3. 对 grasped → grasped 的 nominal retarget，根据实测手指与已保存驱动
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

相关 16 个已有/故障回归检查通过（0.59 秒）；没有展开跨样本测试。
代码结构可复用，第二个 sequence 的 transfer 尚未验证。

## 当前边界

A1 已有局部有效 padded/core artifact（r58）；A2 尚无有效 artifact。
A1 → A2 的 0.2 秒 cuRobo 连接尚未执行，因此不能宣称连接可行或不可行。
完整初始前缀仍在首段 cuRobo canonical 规划碰撞检查处被拒绝；完整
4.8 秒无 reset rollout 尚未完成。

失败的 A2 实测视频（r60 object-relative proposal，源帧 76–82，6 帧，
30 fps，0.2 秒）：
`outputs/oracle_floating_a2_source82_r60/anchor_1/refinement/object_relative_0/floating_xhand_actual.mp4`。
视频仅渲染记录的 PhysX mesh transforms，不运行 GT 动画或额外模拟。
