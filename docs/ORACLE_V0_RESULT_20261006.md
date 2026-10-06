# Oracle V0：首条完整物理轨迹通过

2026-10-06。当前验收使用 **floating XHand**，机械臂未参与这一轮执行。

## 结果

Hopper 作业 `650437.hopper-m-02` / `oracle_connected_r110`：从任务初态执行完整候选后，导出三个 Interaction Anchor 与四个 cuRobo bridge，再启动新的 PhysX 世界，由 artifact runtime 独立执行。两次完整执行均通过。

- 源时间与实际仿真时间：**4.8 s**；144 帧，30 fps，120 Hz physics。
- 七段全部通过；段间 reset：**0**；没有中途设置机器人或物体状态。
- 最终碗已释放：位置误差 **6.49 mm**，旋转误差 **12.73°**，线速度 **0.0251 mm/s**。
- `full_pipeline_success=true`，`task_success=true`。
- Oracle schedule 直接输入；没有 VLM；没有完整 EgoEngine robot reference 输入。
- cuRobo 只用于 RELAXED；两个持物 bridge 的手指 drive target 变化均为 **0 rad**。
- H200 / interactive；申请 10 分钟，实际作业运行 **2 分 32 秒**，退出状态 0。

[完整真实状态视频](evidence/oracle_v0_20261006/floating_xhand_actual.mp4) · [机器摘要](evidence/oracle_v0_20261006/summary.json) · [最终 runtime 报告](evidence/oracle_v0_20261006/pipeline_result.json) · [实际状态 trace](evidence/oracle_v0_20261006/actual_trace.json)

视频为 144 帧 / 30 fps / 4.8 s。画面来自 PhysX 记录的真实网格变换，MuJoCo 仅用于渲染，没有重新模拟或用 reference 动画代替执行。视频各帧显示对应源时间区间的实测终点。

## 每段时间

| 区间，源帧 | 模式 | 时间预算 | 最终验收 |
|---|---|---:|---|
| 0–35 | cuRobo，接近 | 1.1667 s | 通过 |
| 35–76 | A1，抓取 | 1.3667 s | 通过 |
| 76–82 | cuRobo，持物 | 0.2000 s | 通过 |
| 82–101 | A2，倾倒交互 | 0.6333 s | 通过 |
| 101–111 | cuRobo，持物 | 0.3333 s | 通过 |
| 111–128 | A3，放回并释放 | 0.5667 s | 通过 |
| 128–144 | cuRobo，释放后退出 | 0.5333 s | 通过 |

所有区间沿用 Oracle schedule。没有新增等待，也没有拉长短 bridge 的时间。

## 这一轮发现和修复

### 1. 局部优化结果需要在真实 predecessor 上重新验收

r105 的 MPC 分窗预测能够通过，但拼起来重新 Replay 在第 71 帧失败。修复了从 CORE 起点启动时未记录 entry、因而错误报告 `core_entry_acquired=false` 的问题；随后让局部 Replay、whole-local PPO episode 和候选验收都先真实执行前面的 cuRobo prefix，再接管 Anchor。窗口 MPC 输出只作为 proposal。

r106 的 A1 独立 padded `[29,82]` 与 exported CORE `[35,76]` 均通过。small PPO 已接入，网络为 `[64,64]`、24,293 参数；本次 MPC 修正后的零残差 policy 通过，因此没有继续 PPO 训练。没有证据把旧失败唯一归因于快照中的接触缓存；真实 predecessor 的重复执行消除了对此依赖的验收。

另一个已有修复得到保留：连接优化过的 pose commands 时只加入边界连接的速度修正，保留 nominal feedforward，避免把随机 pose residual 的差分当成大幅速度命令。

### 2. A2 边界需要物理反馈，不能把时间投影后的目标当成已经到达

r106 已通过 A1，但 76–82 bridge 的实际碗位置误差为 **62.93 mm**。为满足 0.2 s 的速度/加速度预算，原 source-conditioned wrist 目标被投影；抓握变形和真实跟踪误差又增加了偏差。

复用现有 `feedback_boundary_target`：从真实 bridge 的物体偏差提出新的边界目标，仍满足同一时间预算、原来的 source 边界位置/旋转条件，再由 cuRobo 与 PhysX 验收。r107 的 A2 入口位置误差降至 **2.67 mm**，随后 A2 与 A3 的连续执行通过。

反馈仅使用下一 Anchor 边界。RELAXED 内没有逐帧追踪人类物体轨迹的成功条件。r107 同时修正了 cuRobo 时间优化配置，不能将这次下降当成严格的单变量归因实验。

### 3. cuRobo 默认时间优化不支持当前移动入口

末段曾报 `DT_EXCEPTION`。固定版本 cuRobo 的 `MotionGenConfig` 明确要求非静止入口使用 `optimize_dt=False`；此前 floating planner 仍沿用默认 True。

现在关闭该时间优化。轨迹优化失败时，可使用 cuRobo graph 生成几何路径，再走同一套固定 source-time 的速度、加速度和碰撞检查。r110 的后两个 bridge 实际使用了这个 graph 路径。不可行路径仍被拒绝。

### 4. 释放末段减速不能丢掉退手位置

原减速逻辑只积分衰减速度，截短了 retarget 的退手位置；r107 对比局部原终点，少退约 **10.6 cm**。

现在保留局部退手位置，在已有 CORE 时间内完成减速。自由手腕的末态方向采用已有角速度的减速结果；手指释放序列保留。没有指定样本的退手方向、坐标或等待时间。

### 5. 物理成功的 release 还要给 planner 一个可用出口

r107/r108 释放后，小指实测网格与桌面仍有约 **0.2–0.5 mm** 重叠，free cuRobo 因此无法从该状态启动。

新增实际出口的几何反馈：依据记录的手部碰撞 cover、真实障碍物姿态以及一 physics step 的实测表面运动，自动提出终点平移修正。修正需要累积，不能每次又从原始未修正终点开始。r110 累积得到 **33.77 mm 向上修正**，该数值是物理反馈的结果，没有写入样本配置。

该修正只改变 Anchor 的命令 proposal，由完整实际执行确认出口可达。原 source 时间、物体条件和手指释放命令保留。Hopper 脚本可在同一短 allocation 内为已观察到的 free-entry 网格碰撞重验少量候选；每个候选启动新的任务初态执行，失败尝试不会拼成成功轨迹。

### 6. 保留完整碰撞 cover，同时避免过大的球模型

前序末段曾生成 13,182 个球；即使关闭 self-collision，cuRobo 初始化仍会构造球对中间矩阵。碰撞边界处按实际三角面细分、再合并完全包含子球的父球，并保留原有边界 clearance。本轮开发例从 13,182 降至 1,400 个球，桌面和物体的正 clearance 保留。没有删除手部或持物几何，也没有降低碰撞阈值。

## Anchor artifact 的证据口径

| Anchor | CORE | Synthesis context | Entry |
|---|---|---|---|
| A1 | 35–76 | 29–82 | 真实初态 → cuRobo |
| A2 | 82–101 | 76–107 | A1 actual exit → cuRobo |
| A3 | 111–128 | 105–134 | A2 actual exit → cuRobo |

A1 另有独立的 padded/core 开发验收。最终导出的三个 artifacts 均以完整执行中的实际邻接 cuRobo 区间作为 pre/post context，再由新的 artifact runtime 复验 CORE 与所有交接。`validation.padding_execution` 明确记录了这一点；不要将这份 evidence 解读为三个独立 floating-wrist padded MPC/RL 训练实验。

所有入口来自实际 predecessor rollout，`entry_origin=predecessor_rollout`。不存在用 TACO 中途物体 pose 构造入口并标成物理验证成功的步骤。

## 可复用性和当前边界

修复由 Oracle 区间、接触模式、模型几何、实测状态和时间预算驱动。实现没有针对 `20231024_253` 或 pour/put-in 名称的条件分支；实验输入仍正常指定当前数据、schedule 和物理 artifact 路径。

当前已验证这一条 sequence 的浮空 XHand Oracle V0。其他 sequence 的 transfer、多解，以及 mounted arm 执行尚未验证。assembly self-collision 沿用当前关闭设置。task success 按已有合同验收 Anchor 交互、碗的最终位姿、释放与稳定，不包含真实物料/流体转移的仿真证明。

## 复现入口

`hpc/run_oracle_v0.pbs`，`interactive` / H200 / 10 min。

成功运行的关键参数：

```text
MULTIPATH_CONNECTED_ORACLE=1
MULTIPATH_CONNECTED_FINALIZE=1
MULTIPATH_EMBODIMENT=floating_xhand
MULTIPATH_SCHEDULE=configs/oracle/put_in.json
MULTIPATH_PROPOSALS=<ROOT>/outputs/oracle_v0_putin_r10/proposals
MULTIPATH_CONNECTED_SEEDS=<ROOT>/outputs/oracle_connected_r106/local/anchors
MULTIPATH_INITIAL_BRIDGE_RUN=<ROOT>/outputs/oracle_connected_r103/full
MULTIPATH_BOUNDARY_FEEDBACK_FROM=<ROOT>/outputs/oracle_connected_r106/full
MULTIPATH_EXIT_GEOMETRY_FEEDBACK_FROM=<ROOT>/outputs/oracle_connected_r109/full
```

`ROOT=/scratch/e0985859/multipath_v0_fc2da7b`。局部 Anchor 开发入口为 `scripts/develop_reached_anchor.py`；连续候选为 `scripts/develop_connected_oracle.py`；artifact 导出为 `scripts/export_connected_anchors.py`；最终 runtime 为 `scripts/run_oracle_v0.py`。

本轮验证采用实际物理 Replay、边界可达性和完整连续 rollout；另做了 syntax/diff 检查以及 feedforward、退手终点、碰撞 cover 的小型 CPU 归因检查，没有扩展全面测试套件。
