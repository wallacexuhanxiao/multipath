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

## 后续：从局部 grasp 到 A1→cuRobo→A2

上述 r58→r68 canonical 边界不能直接连接。后续采用用户明确指定的
TACO 帧 82 物体边界条件，结合 A1 实测抓持变换，生成 wrist 目标；
不把 A1 出口当成 A2 入口，不沿人类轨迹逐帧约束 RELAXED。
目标提案不是物理 snapshot：只有实际 bridge 到达后，才能保存为
`physically_acquired_state=true` 的入口。已有 r68 A2 core 只作控制种子，
必须在新入口上重新执行。

### 已定位的连接问题及可复用修复

1. **局部成功不一定有可运输的出口。** r71 padded/core 均通过，但碗仍
   几乎被桌面支撑；cuRobo 的 carried-object 球代理穿入桌面 10.68 mm。
   给需要 grasped transport 的 local synthesis 增加同一碰撞代理的出口
   clearance 条件和出口速度条件。它只约束真实出口，不重新构造状态。
   r75 经物理 MPPI、fresh padded 和 core Replay 通过；其实际 core 出口
   距 table 43.76 mm、距 target 20.36 mm，速度比 0.213。
2. **旧 BC tracking error 被错误继承为新规划的全局碰撞膨胀。** r73 从
   实测状态出发，仍被上段 BC 的 127.34 mm 误差膨胀判为初态碰撞。
   新规划用实测起点和实测几何；旧误差保留为诊断。每个执行步继续
   检查实际手和 carried object 几何，不通过改碰撞阈值接受失败。
3. **原 oracle 目标本身在规划代理内碰撞。** r75 的 source-conditioned
   goal 球代理进入目标板 5.38 mm。用当前场景 signed distance 梯度修正
   目标提案，再检查原 5 cm / 30°边界和原时间预算。r76 修正后的目标
   仍仅偏离 source 9.01 mm / 13.61°，cuRobo 几何规划成功。
4. **移动边界不必沿同一几何路径切线。** r76 随后在时间拟合失败；
   旧标量 path-speed 参数化不能表达当前入口、出口的速度方向。
   commit `55243c6` 增加 endpoint-constrained geometric fit：固定两端
   q/dq，在原 physics ticks 内最小化对 cuRobo 路径的偏移，并硬约束
   命令速度/加速度。拟合后每个配置仍须通过 cuRobo collision 检查，
   然后接受连续 PhysX 验证；拟合成功不是物理成功。
5. **冷节点 native import 阻塞。** r69/r74 分别在 NumPy/Torch native
   import 超时，尚未进入 physics。这两轮不能算 interaction failure。
   加入原 native 文件预读并减少数学库线程；r75/r76 已进入实际运行。
   当前证据不单独证明预读是速度提升的唯一原因。

这些修复依赖实测状态、时间预算、碰撞几何与 grasp contact mode；没有
sequence/frame 特例，没有改摩擦、控制增益或源时间。第二个 sequence
尚未验证，不能声称已证明迁移收益。

### 精简实验记录

| 轮次 | 结果 | 提供的新信息 |
|---|---|---|
| r70 | core 帧 70–73 物体偏差失败 | 仅整形出口命令不足以保持 interaction |
| r71 | A1 local padded/core 成功 | 出口仍需支持于桌面，不能直接 transport |
| r72 | A2 padding 帧 79 grasp retention 失败 | 不再靠 BC 重做 RELAXED transport |
| r73 | 初态 world collision | 旧 tracking inflation 和真实低 clearance 两个原因 |
| r75 | A1 local 成功；bridge GRAPH_FAIL | 已有真正可运输出口，目标代理碰撞 |
| r76 | cuRobo 几何成功，时间拟合失败 | 移动边界切线条件不成立 |
| r77 | 规划与时间拟合通过，执行中 proxy collision 失败 | 固定时长可行；需区分代理碰撞与实际穿透 |
| r78 | 源帧 77.75 proxy collision | 真实碗表面未到达目标板，仍是代理的保守重叠 |
| r79 | 初始化退出 124，尚未 physics | 依赖预读被错误放在关键执行路径 |
| r80 | 源帧 79.25 retention 相对旋转超限 | 首次走过碰撞误判，接下来验证抓持假设 |

r77 job `649000.hopper-m-02`，r78 job `649022.hopper-m-02`，
r79 job `649038.hopper-m-02`，r80 job `649045.hopper-m-02`：
interactive / H200 / 20 分钟。
提交参数请求 32 GB，但 r78 PBS 记录的实际 resource request 为 225 GB；
应以调度器记录为准，而不能把提交参数当成实际分配。
只新增一个针对实际发生过的移动边界失败的快速 regression，连同原
边界检查共 2 个检查通过。当前真实边界的 CPU 必要动力学检查通过：
0.2 秒，命令速度比 0.615、命令加速度比 1.000；这不是物理验收。

本次 bridge 开发从 r75 真正 acquired 的帧 76 snapshot 恢复一次，
之后 bridge→A2 core 不允许 reset。它是源帧 76–101 的连续开发切片，
即使通过，也不等于完整 initial→4.8 秒 Oracle V0 已经通过。

r77 用时 40.85 秒；拟合后 command velocity ratio 0.627，acceleration
ratio 1.000，全部计划配置通过 cuRobo collision 检查。真实执行到源
帧 77 后，在下一次记录前发生代理碰撞；没有到帧 82，没有运行 A2。
录制的帧 77 碗表面到目标板 box 最近距离为 39.66 mm，而旧代理只剩
3.92 mm。这不是已证实的真实表面穿透。

根因之一是 `cover_points` 原来无条件填满 PCA 包围盒，空的格子也
生成球。commit `a0e6a4f` 改用三角面 bounding cells：每个三角面都
被保守 cell spheres 覆盖，空 cells 不再代表实体。没有减小球的覆盖
半径、没有关闭 collision gate。carried-object 代理在物体坐标系缓存，
由 planner、出口检查和每步实际验证共用，避免三处覆盖不同几何。
帧 77 同一碗的代理到 target 距离因此为 9.28 mm；是否足够完成真实
transport 必须看 r78。新增一个针对空体积被误填充的几何 regression，
连同边界检查共 3 个快速检查通过（0.52 秒）。

另外，A1 actual exit 的碗相对 wrist 速度仍有约 0.103 m/s、3.09 rad/s。
这作为潜在抓持滚动/滑动诊断记录；没有在无硬件依据下新增拒绝阈值。
若桥段再次失败，需根据精确失败步的 retention、表面距离和状态定位，
不能把代理修复直接当成 interaction 已解决。

r78 49.18 秒完成，首次代理失败精确在源帧 77.75：target clearance
-2.69 mm。但同一步的真实碗表面顶点离 target box 30.15 mm，离 table
53.55 mm；精确 triangle/box SAT 检查也没有相交。抓持相对变化为
10.13 mm / 0.199 rad，仍满足已有 retention 条件。

因此 `eeddeec` 保留规划层的保守球代理；执行层在代理重叠时，使用
实际 world mesh 做 triangle/box SAT，并对闭合物体补包含检查。真实
相交仍拒绝，代理重叠仅触发精确复核。没有改距离阈值或关闭碰撞。
检查结果逐步保存到 `proxy_overlap_refinements`，真实失败的实际状态
单独保存；不再只留下一个无法 attribution 的 collision 字符串。
这项修复适用于一般 mesh/box，没有任务名、样本或帧分支。

r79 在 hopper-27 预读 154 个 native 依赖时超时，PBS exit 124，wall time
187 秒，尚未生成 bridge log。此前预读作为性能优化，却因 shell
fail-fast 成了强制前置 gate，反而阻止真正仿真启动。`3b007d7` 把预读
改为 20 秒的可选优化；未完成则继续由原受监控 import 验证依赖。
增加每个文件的开始/完成日志，便于定位共享盘读取卡点。没有把依赖
执行迁到 noexec 临时目录，也没有取消真实 import 的 watchdog。

此外，development restore 后 INITIAL 的 USD visual transforms 可滞后
于 native PhysX 状态；bridge 视频改用执行后的 N 个 endpoint mesh
capture，对应相同 N 个 source-time interval，保持 30 fps。源帧 76
的原 stale visual capture 不作为物理初态碰撞证据；初态以 acquired
snapshot 的实际 object pose 为准。

## 本轮最终结果：r80

r80 / `649045.hopper-m-02` 在 hopper-17 真实执行，计算耗时 92.43 秒。
cuRobo 几何与固定时间拟合通过；命令速度比 0.627、加速度比 1.000。
6 个代理重叠步经实际 mesh/box 精确复核，均没有相交。

执行到源帧 79.25，已走 13 个 physics ticks / 0.10833 秒后停止：

- hand-object 相对位移 19.59 mm，未超过原 50 mm 条件；
- 相对旋转 0.35832 rad / 20.53°，超过原 0.35 rad / 20.05° 条件；
- 原生 PhysX 接触仍同时存在：中指约 11.94 N、拇指约 12.89 N；
- 未到源帧 82，所以 A2 core 尚未执行；没有生成新的有效 A2 artifact。

**不能把这个结果描述成碗掉了，或已经发生真实碰撞。** 当前失败是
既有“抓持相对变换近似保持”的条件；0.35 rad 是 project constraint，
不是硬件极限。此前 source object 的 30° 容忍度是另一项条件，未改动。
本轮保留原阈值，也不因为只差约 0.48° 就把结果改为通过。

下一项有信息价值的最小诊断是：保持失败标志，观察完整 0.2 秒末端的
actual contact、actual mesh collision 和 source 帧 82 物体边界。若实际
interaction 失败，再把真实 transport rollout 纳入局部 grasp 优化；
不先围绕 20.53°→20.05° 开新一轮阈值/平滑微调。本轮尚未跑这个诊断。

bridge 启动时只从 r75 acquired snapshot 恢复一次，执行中无 restore
调用；此项由保存的运行代码和实际 source clock 追溯。r80 原报告未
输出 restore counter，已在 runner finally 补上以便后续直接记录。
机器 summary 将这项明确标作 code-derived，不冒充当轮计数测量。

**当前验收状态：A1/A2 局部有效；A1→cuRobo→A2 尚未通过；完整 4.8 秒
Oracle V0 未完成；没有开展 multi-path。** 所有修改都是一般边界速度、
mesh 几何和运行机制，没有样本/任务名分支。迁移到第二个 sequence
仍未验证。

产物：

- [r80 机器结果](evidence/oracle_floating_20261006/bridge/r80_result.json)
- [r80 summary](evidence/oracle_floating_20261006/bridge/r80_summary.json)
- [r80 actual trace](evidence/oracle_floating_20261006/bridge/r80_actual_trace.json)
- [r80 首次失败的实际状态](evidence/oracle_floating_20261006/bridge/r80_failed_actual_state.json)
- [r80 失败桥段视频](../outputs/oracle_bridge_physics_r80/bridge/floating_xhand_actual.mp4)
- [r75 有效 A1 padded 视频](../outputs/oracle_carry_exit_bridge_r75/anchor_0/refined_replay/floating_xhand_actual.mp4)

r80 视频是 3 个执行后的实际 mesh capture，30 fps / 0.1 秒，已验证
可解码；是停止前的短片，不是完成了 0.2 秒的 bridge。它与 r75 的
53 帧 / 1.7667 秒局部 A1 视频分开，不拼接成一次 uninterrupted rollout。

证据目录：[bridge 实验报告](evidence/oracle_floating_20261006/bridge/)。
