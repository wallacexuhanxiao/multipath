# Put-In：每个 RELAXED 区间三解

## 当前已验证的结果

基于已有 Oracle V0，在四个 RELAXED 区间分别获得 **3 条局部 PhysX 验证成功的路径**，包括原始路径与两条替代路径。已完成 **2 条完整的 4.8 秒连续 PhysX 轨迹**：原始 V0 与新替代轨迹。**本轮第三条完整组合未通过**；局部三解不能当作三条完整轨迹成功。

当前使用 floating XHand。机械臂未参与本轮规划或物理验收；没有 VLM、完整 EgoEngine robot reference、源时间拉伸或段间状态重置。

| 区间，半开帧索引 | 接触模式 | 固定时间 | 局部解数 |
|---|---|---:|---:|
| RELAXED [0,35) | free | 1.167 s | 3 |
| A1 [35,76) | grasp interaction | 1.367 s | — |
| RELAXED [76,82) | grasped | 0.200 s | 3 |
| A2 [82,101) | tilt interaction | 0.633 s | — |
| RELAXED [101,111) | grasped | 0.333 s | 3 |
| A3 [111,128) | return/release interaction | 0.567 s | — |
| RELAXED [128,144) | free | 0.533 s | 3 |

新完整轨迹来源：`oracle_diverse_r122/full_solution_1`，Hopper 作业 `651384.hopper-m-02`。`full_pipeline_success=true`、`diverse_solution_success=true`、`task_success=true`；实际时长 4.8 s；7 个段全部通过；段间重置 0。终态碗返回并释放，位置误差 **2.90 mm**，旋转误差 **8.87°**，线速度 **0.116 mm/s**。

这里的 task success 是 Oracle 交互序列与实际终态合约的成功，未模拟碗内液体或物料转移；也不证明 mounted arm 可执行或 assembly self-collision safety。

## 实际路径差异

以下数据来自两条成功的完整 rollout，而不是各段独立开发结果。距离是与原始路径的空间最近距离，不是 source-time tracking 误差；不作为人类轨迹相似度验收条件。

| RELAXED | 平均路径距离 | 最大路径距离 | 实测最小虚拟障碍间隙 |
|---|---:|---:|---:|
| [0,35) | 47.25 mm | 94.51 mm | 12.78 mm |
| [76,82) | 2.79 mm | 3.83 mm | 0.332 mm |
| [101,111) | 13.39 mm | 24.51 mm | 4.04 mm |
| [128,144) | 9.44 mm | 16.98 mm | 1.62 mm |

0.2 秒桥的空间余量很小，三解之间只有毫米级差异。较长区间可以形成明显弧线。保持原始时间与规划预算时，不能承诺每个区间都能形成同样幅度的绕行。

## 做了什么、发现了什么、如何修复

1. **各区间独立累计路径排除。** 障碍由已成功的 wrist 路径自动生成，不共享到其他 RELAXED 区间。尺寸同时受区间距离、速度/加速度预算和时间约束；沿路径使用小→大→小的轮廓。人工障碍只约束 wrist，真实 hand/object/world 碰撞模型继续参与规划与物理检查。
2. **固定时间拟合必须保留绕行。** 发现 cuRobo 的几何绕行会被无约束时间拟合抹掉。修复为在原始时间预算内加入虚拟障碍分离约束，保持实测初始速度及终端驱动速度，重新检查拟合后的实际几何。
3. **移动边界需要可达空间。** 发现短桥端点附近的排除盒会阻塞本来必须先沿原速度移动的路径。根据端点速度相对弦方向的偏差与加速度预算，自动保护共享边界邻域；旧路径内部必须继续被非空障碍排除。
4. **规划曲线和实测曲线分开。** 抓着碗时 wrist target 与 actual wrist 有显著偏移，不能只凭命令曲线证明多样性。每个物理步采集实际 wrist，验收实际曲线；如果缩小虚拟障碍，记录原 proposal、缩放及原因，并要求每条旧曲线仍被排除。物理约束与时间预算不变。
5. **用实际抓握关系连接交互。** 新入口可能产生不同的 hand-object transform。桥目标保持同一个 canonical tool boundary，通过实测 transform 换算 wrist target；后续 core 也基于实测 grasp 生成命令。手指 drive/preload 在抓握桥中冻结，不能冻结接触压缩后的 measured finger q。
6. **每个新后继从真实前驱重算。** 独立成功的局部桥，接到另一条完整轨迹中，入口 q/dq、物体速度和 grasp 可能改变。后继必须使用真实出口重新规划并物理执行。只有完整 measured state 与 controller memory 一致时才重用已经验证的命令。
7. **保留有证据的成功前缀。** r118 已真实通过初态→A1→短桥→A2；复用的是它的驱动命令，再从初态实际执行，未恢复中间状态。修复了 prefix 变量与布尔标志同名导致的异常，以及旧 planning success 标志泄漏到新失败段的问题。
8. **平滑代价作为搜索策略。** 全局替换成 jerk 代价在 r119/r120 反而恶化短桥接触。恢复已有成功轨迹，只把平滑代价作为局部候选，最终以真实抓握/碰撞/交互结果决定。
9. **修复后半段。** r118 在 [101,111) 的 frame 106.5 丢失对向接触。r121 从已经实际通过的 A2 出口重新开发该区间，同时执行 A3；r122 将修复接入完整轨迹并通过。

以上机制不依赖 sequence 名称或手写当前样本帧索引。尚未完成第二个 sequence 的 transfer 验证，因此应表述为可复用机制及本样本物理证据，不能声称已证明跨样本收益。

## 第三条完整轨迹为什么失败

r122 的第三条候选完成了第一条新 free bridge 与 A1，A1 交互成功。随后在 [76,82) 的 nominal boundary fitting 被拒绝，没有进入这段桥的 PhysX 执行。

A1 实测出口 Y 速度为 **−0.02885 m/s**，而 grasp-conditioned 的后继驱动目标在约 **+0.06916 m** 处，必须在 0.2 s 内到达。现有 nominal 预算无法同时满足位置与两端速度。即使换成 canonical A2 的实测 pose/twist，仍不能通过固定时间拟合。

这说明 **局部 interaction success 不保证 outgoing bridge compatibility**。拒绝这个组合是合理结果；不能把 nominal acceleration 当作实际 PhysX 硬件限制，也不能通过延长桥时间或赋值到 A2 状态使它通过。

当前修复路径：`anchors/bridge_boundary.py` 与 `scripts/refine_diverse_anchor.py`。从真实 alternative prefix 获得 A1 入口，计算下一段固定时间边界需要的速度调整，在前一个 Anchor 内生成小型末端切向残差候选。源时间、终端命令位置、手指驱动保持一致；候选通过真实 interaction + outgoing bridge 执行后，再从初态重新取得入口验证导出的 core，最后跑完整 trajectory。

**r123 / `651413.hopper-m-02` 的局部结果已通过**：两个候选即可取得 valid padded interaction、实际执行成功的 0.2 s outgoing bridge，以及从新鲜物理前驱取得入口后执行成功的 exported core。手指命令、终端 wrist 命令位置及源时间均未改变。实际出口 Y 速度从 −0.02885 变为 +0.13073 m/s，后继桥对两条旧路径的实测排除间隙 0.297 mm。导出遗漏 `source_fps/control_dt` 的字段错误随后修复；这些元数据由实测 core 边界时间与命令数量恢复，没有修改状态或 success flags。

**r124 / `651420.hopper-m-02` 已连续通过全部三个 Anchor 与前面三个 bridge**，实际执行到 4.267 s。终段 [128,144) 在几何 waypoint 检查被拒绝。诊断发现：seed 的平移来自 measured-boundary fit，但其旋转仍直接取旧驱动轨迹；第一处共同 waypoint 对 table 的球代理间隙 −1.84 mm。使用同一条 boundary fit 的旋转后，该点间隙 +3.68 mm。修复只在 V1 detour seed 中统一平移与旋转的来源，未放宽碰撞判定或改变已通过的 V0 backend。

r125 / `651435.hopper-m-02` 从初态重跑完整第三条，保留 r124 的成功前缀命令；在 4.325 s / frame 129.75 被实际手部 mesh 碰撞拒绝。几何 seed 修复确实移除了规划阶段的错误，但不能保证 closed-loop 实测几何安全。没有取消该物理 gate。

r126 / `651446.hopper-m-02` 从真实 A3 出口，仅重搜最后一个区间，保留前一条完整成功路径作为 incumbent。24 个候选搜索结束，没有 accepted 第三解；当前入口对应的 terminal bank 为 2 解，原始独立开发 bank 仍为每段 3 解。尝试 17 完整执行了局部终段并通过实际路径排除，但因为长度 0.2273 m / 弦长 0.06553 m = 3.47，超过现有 3.25 弦长的弧线质量门槛而被拒绝。它没有被标记为完整 trajectory success，也未取得最终 task acceptance。

**下一轮剩余问题**有两个：实际 A3 出口下 free hand 的 transient geometry / tracking 安全；以及非零入口速度下，固定“路径长度 / 弦长”是否适合作为 hard diversity gate。入口 wrist 速度约为 `[-0.598, +0.330, +0.074] m/s`，必需的转向会增加路程；该门槛需要按真实边界及可达性分析，不能只为这次 3.47 的候选直接调阈值。

额外修复了搜索结果没有 accepted bank 时仍进入完整 rollout 的流程错误：现在 bank 未 complete 时直接结束该 job。后续碰撞报告保存具体 mesh、障碍与 source frame，便于直接定位。本轮收尾时没有继续提交新实验。

## 可直接播放与复现的产物

- 新完整实际状态视频：`evidence/oracle_diversity_20261007/solution_2/floating_xhand_actual.mp4`
- 原始路径与新完整路径上下叠放：`evidence/oracle_diversity_20261007/baseline_vs_solution_2.mp4`
- 两条完整实际路径的分段空间比较：`evidence/oracle_diversity_20261007/full_wrist_paths.png` / `.pdf`
- 每段三条独立局部实际路径：`evidence/oracle_diversity_20261007/local_wrist_paths.png` / `.pdf`，不能当作三条完整 rollout 图。
- 新完整 machine-readable report 与 trace：`evidence/oracle_diversity_20261007/solution_2/pipeline_result.json`、`actual_trace.json`
- 局部 bank：`evidence/oracle_diversity_20261007/local_bank/bridge_bank.json` 及区间命令/实测曲线。

视频是记录的 PhysX mesh transforms 在本地渲染的结果。MuJoCo 仅用于渲染，没有替代物理执行。视频 144 帧、30 fps、4.8 秒；trace 记录 145 个源帧边界，桥段另外保存了 120 Hz 的实际 wrist 路径。

作图时优先使用相同 source clock、相同镜头的上下叠放视频；区间图使用共同空间投影和 mm 单位，并明确区分 local-only 与 successful-full 数据。报告不要从局部三解推断所有组合都可执行。
