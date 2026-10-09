# EgoEngine residual PPO 对齐记录（2026-10-09）

依据：用户提供的 Appendix 截图、本地 `egoengine.pdf` 第 20–23 页，
以及 Sec. 3.2.2。对应代码已推送到 `wallacexuhanxiao/multipath`。

## 本次实际改动

| 项目 | 修正后的实现 |
|---|---|
| Residual action | 固定的局部 retarget + wrist/finger residual；PPO 不再在线用 acquisition controller 重写基线 |
| Observation | 实测手状态、工具状态、局部 retarget 命令与源目标；不需要完整机器人 reference |
| Object reward，C.3 | `C - sqrt(lambda_R * e_R² + lambda_p * e_p²)` |
| Early termination，C.4 | 同一 weighted error 超过 C 即终止；不继续越过失败帧训练 |
| Contact，C.7 | 实际 thumb AND 至少一根 non-thumb 接触才得到 bonus |
| Lifting，C.8 | 需要向上交互时使用实际 `lambda_z * (z - z_initial)`；初始高度来自真实局部入口 |
| TACO setting | human-mimic = 0，smoothness = 0，无 domain randomization |
| Evaluation，C.9 | 只累计 object tracking；contact、lifting 和 bridge penalties 不计入 evaluation_return |
| Network | 独立 64×64 actor / critic；约 2.4 万参数，不使用 Appendix D 的 HPT 模型 |
| Residual noise | 三类共用线性 multiplier，1 → 0.25；同一 PPO batch 内固定，保存到 checkpoint |
| Mode switching，C.1 | local Replay → MPC → PPO；H=20、最多两 chunk lookahead，整个 prediction 通过后才提交一个 chunk，再重选 mode |

实际 boundary / collision / retention / task checks 仍保留。
它们决定物理验收，不额外加入 reward。RELAXED 的 cuRobo backend
和源时钟没有修改；新逻辑不按 source ID 或动作名称分支。

## 论文数值与工程选择

论文明确给出公式与 TACO/Aria 的配置区别，但没有给出 TACO 的
精确 contact/lifting 系数、PPO 网络大小或噪声 schedule 数值。
目前所有类型共用：C=1，lambda_p=400，lambda_R≈3.64756，
contact=2，lambda_z=10/m。这些是公开标记的工程选择。
5 cm / 30° 是 C.4 椭圆边界的轴截距，位置与旋转同时偏离时，
按组合误差验收。没有复制截图中的 pour 专用 threshold。

## 物理证据与当前边界

- r199：412.24 s，三类首个 Anchor 均失败。brush 某预载候选
  保留了出口 opposition，但工具仅升高约 3.7 mm。
- r200：574.90 s，新的 C.3/C.4/C.7/C.8/C.9 已实际进入 PhysX
  PPO，三类首个 Anchor 仍未通过，完整 rollout 数为 0。
- r201：job `656589.hopper-m-02`，interactive H200，申请 20 min，
  实际 563.78 s；固定 local retarget residual 与共同 noise schedule
  已实际训练、执行，三类首个 Anchor 仍失败，完整 rollout 数为 0。

| Source | 零 residual 失败帧 | 最佳确定性 PPO 失败帧 | 最佳回放的物理证据 |
|---|---:|---:|---|
| pour 246 | 36 | 38，object tracking | 没有 thumb/other 同时接触；工具升高约 12.4 mm，不能计为有效 grasp |
| brush 057 | 30 | 32，core exit handoff | 9 帧双侧接触；工具仅升高约 1.9 mm，出口状态不满足交接 |
| hit 003 | 34 | 34，object tracking | 没有双侧接触；工具高度变化约 0.09 mm |

表中采用 `refinement/ppo` 的确定性评估。每类该阶段实际执行
60 次更新、253 次 episode；此外重新执行真实 predecessor 的
whole-local PPO 也失败。没有将接触、工具被推起或 training return
提高当作 Anchor 成功。实测记录与汇总见
`evidence/oracle_action_types_20261008/r201_ppo_diagnosis.json`。

本轮还关闭了 local solver 的旧 `allow_physical_prefix` 捷径：
即使前 20 帧物理通过，只要后一个预测 chunk 失败，就不能提交
前缀或推进到下一状态。这与 Appendix C.1 一致。r201 所有候选
都在第一个 20 帧前停止，因此这项后续修正不改变 r201 的失败结论；
本轮没有新的完整 lookahead 物理通过证据。

CPU 检查确认共同 tracking 边界、training/evaluation 分离、
三类实际入口的首步命令连续性、优化器更新和 noise checkpoint。
这些检查不是 physical acceptance。成功仍需 fresh padded/core
以及 uninterrupted full rollout，不用参考动画代替实际执行。

当前 brush 实际状态诊断视频已下载、渲染并检查首尾帧：
`outputs/oracle_action_types_20261008/r201_diagnosis/brush/fresh_local/floating_xhand_actual.mp4`。
它是 12 帧、30 fps、0.4 s 的失败局部片段，不能当作完整任务视频。

## 可复用范围

修改作用于共同的 reward、PPO 执行接口与训练流程。局部 retarget
只使用该 Anchor 的 raw 数据和已有几何拟合，坐标注册只修改
命令；没有设置物体 pose，也没有构造 reference-only 的中途状态。
pour、brush、hit 使用相同实现，后续 source 也走这一入口。
