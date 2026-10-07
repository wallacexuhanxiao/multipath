# Oracle Anchor + cuRobo：新增 TACO source 实验报告

更新：2026-10-08。本轮目标是换不同的 TACO 源视频，运行完整 pipeline，
并把通过验证的实际状态视频下载到本地。

## 当前结果

| TACO source | 源时长 | 完整连续 PhysX | 结果 |
|---|---:|---|---|
| 20231024_248 | 4.200 s | **通过** | 三个 Anchor、四个 RELAXED、任务完成、段间重置 0 次 |
| 20231024_258 | 4.267 s | 尚未通过 | A1 成功；首个持物桥段的下一 Anchor 入口角度仍超标 |
| 20231024_243 | 4.300 s | **通过** | 半径接口修复后返回 bridge 通过；局部 A3 residual PPO 修复后完整复验成功 |

`248` 的成功来自 r152a / Hopper `653289.hopper-m-02`。端到端程序自动完成
准备、连续候选、局部 artifact 导出，再从任务初态独立运行最终验证。
端到端耗时 144.465 s；实际任务执行时间保持 4.200 s。

末态位置误差 0.01062 m，角度误差 0.22867 rad（13.10°），速度
0.0001002 m/s，碗已释放。任务成功来自实际 simulator state。

| 段 | 源帧区间 | 原始时间预算 | 实测结果 |
|---|---|---:|---|
| RELAXED 0 | [0, 26) | 0.8667 s | 通过 |
| A1 | [26, 65) | 1.3000 s | 通过 |
| RELAXED 1 | [65, 74) | 0.3000 s | 通过 |
| A2 | [74, 89) | 0.5000 s | 通过 |
| RELAXED 2 | [89, 96) | 0.2333 s | 通过 |
| A3 | [96, 120) | 0.8000 s | 通过 |
| RELAXED 3 | [120, 126) | 0.2000 s | 通过 |

三个 Anchor 均有 `padded_rollout_complete=true`、`core_replay_success=true`、
`entry_origin=predecessor_rollout`、`handoff_verified=true`。最终视频使用
连续 PhysX 实测轨迹和本次原生场景几何离线渲染；没有用参考动画代替执行。

## 本地视频与证据

文件夹：
`outputs/oracle_end_to_end_20261007/successful_sources/oracle_e2e_r152a_20231024_248/`

- `source_vs_actual.mp4`：原 TACO 视频在上，实际状态视频在下，同一 source clock。
- `final/floating_xhand_actual.mp4`：完整实际状态视频。
- `final/actual_trace.json`：连续实测状态。
- `final/pipeline_result.json`：最终物理验收报告。
- `result_summary.json`：精简的段落、Anchor 与任务证据。

两段完整视频均为 126 帧、30 fps、4.2 s，已完整解码并抽帧检查。

第二条完整新 source `243`：
`outputs/oracle_end_to_end_20261007/successful_sources/oracle_e2e_r156_20231024_243/`。
同样包含 `source_vs_actual.mp4`、`final/floating_xhand_actual.mp4`、完整实际
轨迹与报告。129 帧、30 fps，source 与实际均为 4.3 s。

r156 / `653339.hopper-m-02` 的独立最终验证：所有七段通过、三项 Anchor
通过、段间重置 0 次、task success。末态位置误差 0.00645 m，角度误差
0.06483 rad（3.71°），速度 0.00002195 m/s，已释放。整条 pipeline
耗时 358.0 s，包含观察失败、局部优化、重新连接、导出和独立最终验证。

该 source 的 CORE 区间为 A1 [23, 61)、A2 [70, 97)、A3 [105, 123)；
四段 cuRobo 区间 [0, 23)、[61, 70)、[97, 105)、[123, 129)。

## 找到的问题与可复用修复

### 1. 局部验证和最终执行的初态准备不同

原来局部验证通过的同一组命令，接进连续执行后仍会失败。`258` 的
CORE 入口姿态只差约 6.9e-5，之后却逐步放大。原因是构造场景与局部
episode 的物理初态准备走了不同路径，驱动历史没有完全一致。

统一为同一个物理初态准备函数。准备发生在任务计时开始前；段间不
增加等待，不重置对象或手。此修复后 `248` 完整通过，`258` 的 A1 也
首次在连续执行中通过。代码不包含 source 名称判断。

### 2. 迁移局部交互时，误把实测姿态当成驱动目标

实际抓持下，物体负载和手指接触会让 measured pose 与 drive target
存在稳定偏差。使用 measured pose 替代已验证 drive command，会改变
接触预载。现在迁移局部交互的驱动命令，并保留原先验证过的控制器
语义；实测状态用于物理可达性和交接验证。

同时，持续抓持的手指约束只用于 grasped → grasped Anchor。获取抓持
和释放 Anchor 保留相应动作空间，不把一瞬间的接触误当成稳定抓持。

### 3. 持物边界反馈只修位置，没有正确解决角度和速度

`258` 的首个 carry bridge 保持抓持，位置反馈能把边界误差压到毫米内，
角度却一直约 35°。r153 把反馈扩展到完整位姿后，仍发现固定人类参考
入口速度限制了可达旋转范围；十次重跑没有新收益。

修复为在固定源时间内联合选择 canonical 入口姿态和速度。Oracle
的物体位姿、接触语义、时间与 30° 条件保持不变；规划预算仍使用配置
里的原值。真实入口仍必须由 PhysX 执行到达。CPU 边界分析得到预测
角度误差 0.4983 rad（28.55°）；这只是候选，尚未构成物理通过证明。

### 4. target 包围盒和粗碰撞球误挡真实空隙

`243` 的 A2 出口曾被 target 包围盒判为 -0.15 mm 碰撞，但同一实际
状态的三角网格碰撞检测显示无碰撞，间距 42.18 mm。现在 target 与
自由工具都使用本次场景的实际网格，不用实心包围盒填掉容器空隙。

r153 随后暴露：原来的边界碰撞球细化仅处理 table cuboid，未处理
mesh obstacles，导致返回桥段报告 `Start or End state in collision`。
新增按 cuRobo 实际 mesh world 细化近边界的表面覆盖；保留每个三角形
的覆盖，先检查实际网格是否真的相交，碰撞条件没有被关闭。

### 5. 端到端程序遗漏了 cuRobo 的失败类别

已有自动修复能处理固定时间拟合失败，却把 `GRAPH_FAIL` 直接作为
无法修复退出。现在实际 carry 规划失败可以进入前一个 Anchor 的局部
重合成流程，使用已记录的实际 predecessor 和 bridge context。

## 后续物理验证结果

r154 / `653314.hopper-m-02`：`258` 实际边界角度仍约 33–37°，预测改善
没有转化成物理成功。`243` 的 A2 padded/core 已通过，但返回桥段重复
`GRAPH_FAIL`；碰撞球细化没有触发，因此不能把这一失败继续归因于
已证实的包围盒误挡。停止了无改善的重复运行，失败证据保留。

继续发现反馈模型的坐标系错误：抓持偏差应保留在手的局部坐标系，
不能统一作为世界坐标的物体偏移。现在预测采用
`world wrist tracking correction × candidate wrist × measured hand-object transform`。
同时为 planner 失败记录 bounds/world/self 的实际约束分量，避免把
graph 的泛化 “in collision” 警告直接当成碰撞根因。

r155 / `653324.hopper-m-02`：`258` 仍未通过，实测角度约 32.4°，下一步
需要使前一个 Anchor 的出口与实际 carry 动态兼容。正确坐标系是接口
修复，但该轮没有证明它足以解决此 source。

`243` 的 planner 约束诊断确认：entry 的 bounds/world 均为 0；goal 的
bounds 为 0，world collision cost 为 259.8049。直接检查所用 cuRobo
`warp_sdf_fns.py`，发现 `compute_esdf` 返回球心到 mesh 表面的距离，
只有 collision cost 分支才加半径。我们的 clearance 查询因此漏掉半径，
错误地把实际 planner 拒绝的 goal 投影成“安全”。

修复为明确查询半径为零的点，再对世界联合距离加入实际碰撞球半径。
table 与 mesh 使用同一约定，goal projection、边界表面细化和 measured
clearance 与 planner 的球碰撞含义一致。保留 planner 与实际三角网格
检查，不放宽碰撞阈值。

r156 / `653339.hopper-m-02`：修复后 `243` 返回 bridge 真正通过，连续
执行推进到 A3 第 122 帧，首次暴露末次释放的实际物体轨迹偏差。
程序自动进入 reached-entry 局部 Replay → MPPI → 小型 residual PPO；
第 6 轮 PPO 训练得到通过物理检查的 sampled command trajectory，再通过
fresh padded/core、连续候选与独立最终 rollout。这里验证的是导出的
固定可执行命令，不声称 mean policy 已收敛。没有调宽源时间、角度
容忍或碰撞条件。

`258` 在三次候选后仍明确失败，实测入口角度 0.5852 rad（33.53°）。
保持 30° 验收条件，没有输出成功视频。离线几何分析发现可以在原 A1
CORE 语义条件内提出约 4° 的出口调整，但该 proposal 尚未经过物理
验证；它不构成成功证据。剩余问题是这个 source 的 Anchor 出口与真实
carry 动态兼容性。本轮已得到两条不同新 source 的完整成功视频。

## 证据边界

- Oracle schedule 由人工标注，runtime 无 VLM、无自动分段。
- 只合成局部 Anchor；RELAXED 由 cuRobo 生成，不跟踪人类逐帧路径。
- 使用经过物理验证的**局部交互库作为初始化**。这不是只靠原始
  retarget 的冷启动收敛证明，也不需要完整 EgoEngine 机器人参考。
- 当前 embodiment 为 floating XHand。mounted arm、自碰撞强化验证和
  流体/内容物转移不在本次成功声明内。
- 本报告记录的是新 source 的单条 V0。2026-10-08，`248` 已补齐每个
  RELAXED 区间三解和三条完整连续执行；见
  [后续多解报告](DIVERSITY_E2E_20261008.md)。`243` 仍是一条。
