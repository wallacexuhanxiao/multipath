# A2 抓持失败：原因定位、修复与物理结果（2026-10-06）

结论：r68 / 648809 已通过 A2 的完整 padded Replay 和 exported-core
Replay。代码提交 `6a2d8b0`；A1→A2 cuRobo 连接与完整 V0 尚未通过。

## 实测事件

r65 整段 PPO 的最佳评分候选在帧 95 失去拇指接触。
取得物理抓持时的 thumb bend drive target 为 1.233 rad。

| 源帧 | thumb bend 目标 | 实测角度 | 目标−实测 | 拇指接触力合计 |
|---|---:|---:|---:|---:|
| 94 | 1.077 rad | 1.021 rad | +0.056 rad | 28.94 N |
| 95 | 0.789 rad | 0.995 rad | −0.206 rad | 0 N |

帧 95 中指仍有约 4.21 N 接触，物体相对手的旋转偏差约 0.072 rad。
这次具体失败是拇指接触丢失，而不是物体已经完全掉下去。拇指弯曲
目标相对取得抓持时退了 0.444 rad，并在丢失接触的帧转为张开驱动。

代码问题：初始 retarget 有 preload 保护，MPC / PPO 残差只检查 URDF
范围与目标速度，能绕过保护，把承载手指主动张开。

## 修复与配对实验

同一保护实现接入 nominal、MPC、PPO 和失败提案投影；仅作用于要求
持续抓持的局部 Anchor，使用真实 native contact 标识承载手指。
取得/释放 Anchor 仍可改变抓持，非接触手指保留调整空间。

r66 / 648804 做了同一 wrist 控制的配对 Replay：位置、旋转、六维
feedforward 与 r65 refined controls 完全相同，仅投影手指目标。
29.23 秒完成，但仍失败于帧 91。拇指 drive target 已保持为 1.284 rad，
中指接触却丢失，剩余拇指接触约 0.17 N。相对旋转 0.447 rad。
因此“禁止拇指张开”并不能单独解决整个 A2，不能据此声称抓持修好。

该结果暴露了第二处代码错误：用 `command_q - actual_q` 的正负当作
关节闭合方向。中指 proximal joint 的 drive error 为负，但正方向
依然是 flexion；旧规则反而禁止它继续闭合。负误差来自接触位移，
不等于负方向能夹紧物体。

现在闭合方向取自 SPIDER XHand 的关节语义，保护承载手指的闭合目标，
允许继续屈曲；index abduction 不按闭合关节限制。这是 embodiment
元数据，不是样本/任务特例。没有 frame 95 分支，没有调整 source 时间、
30° 容忍度、摩擦、PD 增益或 drive effort。

r67 / 648806 在发现方向约束有误后取消，不作为完整物理结果。
r68 / 648809 使用修正方向重新执行完整 local synthesis，105.03 秒完成。
初始 Replay 虽通过入口，仍在帧 86 因 core 物体偏差失败；随后浮空
XHand MPPI 在 3 轮、192 个残差候选中找到 7 个成功候选，另有 1 个
measured-grasp 提案。选定控制经 fresh padded Replay 和独立的
exported-core 验收，均成功。本轮不需要 PPO；PPO 使用相同保护代码。

这不是同一条 wrist 轨迹仅改手指后就成功：r66 是这样的配对实验，
且失败；r68 还进行了物理 MPC 优化。因此可以确认两个代码缺陷已修，
以及修复后的合成流程能完成 A2，不能将全部收益单独归因于拇指保护。

相关 17 个快速检查通过（0.63 秒）；唯一新增回归针对已发生的残差绕过
承载指保护问题，覆盖闭合、继续屈曲、非接触指调整和 release。

## A2 物理验收

| 项目 | r68 实测结果 |
|---|---|
| padded synthesis | 源帧 [76,107)，31 帧 / 30 fps，1.033 秒，成功 |
| exported core | 源帧 [82,101)，19 帧 / 30 fps，0.633 秒，成功 |
| A2 入口目标 | TACO 源帧 82；不是 A1 出口 |
| 实测入口误差 | 3.42 cm / 18.03°，通过既有 5 cm / 30°条件 |
| 入口来源 | r58 真实帧 76 snapshot → 实际 padding → 实测帧 82 |
| core 边界 | 执行 padding 到达，没有在边界设置状态 |
| 抓持 | core 全程通过既有对向接触和相对变换保持检查 |
| core 入口接触 | 中指约 20.68 N、拇指约 19.55 N |
| core 出口接触 | 中指约 40.30 N、拇指约 44.50 N |
| rollout 内 reset | padded 0，core 验收 0 |
| 时间 | 30 fps；没有等待或延长 source 时间 |

snapshot restore 只发生在独立开发 rollout 的起点；这不是一次完整
initial→A1→bridge→A2 的连续验证。`entry_physically_reachable=true`
对应真实 predecessor 加 synthesis padding 的来源链，不代表 cuRobo
已经验证过同一入口。

修复结构不依赖当前 sequence、帧索引或 Put In 任务名；闭合方向是
XHand embodiment 元数据，保护由实测接触和 Anchor contact mode
决定。第二个 sequence 的迁移收益还没有实验验证。

## 当前连接边界

A1（r58）与 A2（r68）都有局部有效 artifact，但 canonical 边界
仍不满足现有 grasped bridge 接口：最大手指姿态差 0.237 rad，
超过该接口的 0.10 rad 预检查；drive target 最大差 0.488 rad。
当前 bridge 冻结抓持手指，无法直接产生新 A2 的这些 hand 边界。
手物相对变换的最大矩阵元素差 0.068，也超过既有 0.05 的交接检查；
对应实际相对位移约 5.98 mm、旋转约 4.31°。

两端 wrist 距离 13.26 cm、旋转差 49.33°，source bridge 预算仅
0.2 秒。这里是边界预检查，还没有执行该 cuRobo bridge；不能据此
宣称物理连接必然不可行。下一步需协调 canonical 抓持边界，再用
固定时间 cuRobo 和实际 rollout 验证；不允许直接设到 A2 entry。
完整初态前缀仍被 cuRobo canonical collision 检查拒绝，完整 4.8 秒
V0 尚未完成。

## 产物

- [A2 padded 实际 PhysX 视频](../outputs/oracle_floating_a2_closure_search_r68/anchor_1/refined_replay/floating_xhand_actual.mp4)
- [A2 core 实际 PhysX 视频](../outputs/oracle_floating_a2_closure_search_r68/anchor_1/exported_core/floating_xhand_actual.mp4)
- [机器报告](evidence/oracle_floating_20261006/r68_summary.json)
- [完整开发结果](evidence/oracle_floating_20261006/r68_result.json)
- [有效 A2 artifact](evidence/oracle_floating_20261006/r68_anchor_1/report.json)

视频按 source 帧率渲染记录的 PhysX mesh world transforms，已检查
31 帧和 19 帧视频可解码；不是 GT/reference 动画。105 秒是计算耗时，
不是机器人执行时长。视频在本地输出目录与 Hopper 结果目录，未纳入 Git。
