# 加速度来源核查与 A1 完整 Replay

## 来源与判定修正

实际运行配置 `vendor/curobo-v077/src/curobo/content/configs/robot/franka.yml:113` 为 `max_acceleration: 15.0`。MountedScene 将其用于局部拟合、规划以及实际 dq 差分验收；最后一项混淆了规划 budget 与实际物理限制。

场景加载 Isaac FrankaPanda USD。Franka 官方 FCI 文档给出的 Panda/FER 关节加速度值为 `[15,7.5,10,12.5,15,20,20]` rad/s²，并在关节空间控制要求中约束控制指令 q_c 的导数。它没有支持当前程序“所有关节实测差分加速度都不得超过 15×1.05”的判定。

来源：[Franka 官方 Control Interface Specification and Robot Limits](https://frankarobotics.github.io/docs/robot_specifications.html)。查阅日期 2026-10-02。

因此：保留 15 作为现有 nominal 拟合/预检查预算，保留实际加速度记录；取消将该 ratio 单独当作 actual-state 失败的条件。这不是把真实硬件加速度阈值改高，也不代表现有命令已经通过真机 FCI 的全部逐关节加速度/jerk 检查。本项目此轮验证的是 mounted PhysX 仿真。

## 第 35–42 步小诊断

无需新 GPU 诊断，直接读取 r20 保存的真实状态。完整八行数据见 `r20_step35_42.csv`，关节索引 6。

第 38→40 步位置误差绝对值从约 0.00618 降至 0.00429 rad；第 40 步实际速度 -1.1871 rad/s、目标速度 -1.1025 rad/s，测量关节力矩由前一步 -0.3652 变为 +0.1186 Nm。没有看到位置误差或测量力矩同时暴涨。手指此前已经在运动，第 40 步不是手指首次启动。

该证据支持取消无依据的统一实际加速度 gate，但不能据此断言完全没有耦合或 overshoot。Isaac measured joint efforts 不是隔离后的实际 drive torque；不把 PD 估算或显式 effort command=0 当作驱动力矩证明。

## 实施范围

- 不新增 smoothing，保持 r20 局部拟合和 source timing。
- 实际 arm 速度检查保留，非有限状态拒绝；实际加速度仍记录，但相对 cuRobo budget 的 ratio 仅作诊断。
- 每帧实际 trace 增加 measured joint efforts；局部报告保存配置中的 drive effort caps。
- 完整 A1 采用 nominal precheck → padded physical Replay → exported-core replay；Replay 失败则进入现有 mounted MPC，随后重新 padded Replay。
- r10 几何、规划和执行控制接口不变；共享实际加速度验收语义的错误已纠正。
- 现有物理验收仍有边界：尚无完整穿透/自碰撞验收；assembly 当前关闭 PhysX self-collision，规划层有 collision checking。此次不把局部交互通过等同于所有硬件/碰撞安全项均获证明。

## 本轮作业

`643336.hopper-m-02`，H200 interactive，上限 30 分钟；输出目录 `/scratch/e0985859/multipath_v0_fc2da7b/outputs/oracle_a1_physical_replay_r21`。

这是完整 A1 synthesis 作业，不是第七次 startup smoothing diagnostic。

本次收尾查询：作业在 hopper-17，状态 R，walltime 约 5 分 09 秒；日志停在 SimulationApp 初始化的 `omni.physx.tensors` 扩展加载（日志相对时间 45.249 秒）。尚无 PADDED_REPLAY_BEGIN 或 development_result.json。原因未确定，不能把它当成交互失败或 MPC 失败。保留该作业及 30 分钟上限，没有迁移或重复提交。

当前结论：加速度判定的来源错误已修复，r20 小诊断已完成；A1 是否能完成交互仍未获得新证据。没有新增 smoothing 或调高物理阈值。3 项既有 focused tests 通过，未扩展测试集。


## r21 初始化停滞调查与处理

此前只反复查看日志尾部、将其当成慢启动继续等是不充分的。后续检查了完整 run.log、Kit 详细日志、PBS CPU/内存统计，并与已成功启动的 r20 对照。

### 已确认

- r21 在 hopper-17。第 45.092 秒 Kit 日志记录 `Initializing plugin: omni.physx.tensors.plugin`，随后长时间没有进展；运行约 13 分钟时累计 CPU 仅 17 秒、内存约 1.31 GB。没有进入 local IK 或 Replay。
- Vulkan `ERROR_INCOMPATIBLE_DRIVER`、`Failed to create any GPU devices` 在成功的 r20 也出现过；r20 随后在 37.782 秒完成 SimulationApp 启动。因此这些错误本身不能解释 r21 独有的停滞，不能直接归因于 Vulkan。
- r21 的单 Anchor 启动入口没有 r20 所带的 CUDA 实际运算检查，也没有初始化超时/栈输出。
- 已保存远端 `run.log`、`kit_startup.log`、`job_at_stall.txt`，随后取消卡住的 r21（最终 walltime 16:46，取消退出码 271）。这不是交互失败。

### 最小处理

在 `develop_mounted_anchor.py` 的 SimulationApp 构造前加入同进程 CUDA 分配、运算和同步，输出 `CUDA_PROBE_BEGIN/READY`；Isaac 构造输出 `SIMULATION_APP_BEGIN/READY`。启动阶段由 180 秒 faulthandler watchdog 保护：超时打印 Python 线程栈并退出；成功构造后取消 watchdog，不限制后续 synthesis 时间。没有改控制器、Replay/MPC、物理参数、任务 gate 或 source clock。

这是修复“初始化无诊断、可空转整个 allocation”的缺陷，同时验证先建立 CUDA context 是否消除阻塞；**尚未证明它修复了底层 PhysX/CUDA 阻塞**。当前没有拿到原进程的 native stack，不能区分驱动调用卡住与插件内部死锁。

### 重跑

r22 / `643351.hopper-m-02`，完整 A1 Replay → 必要时 MPC，H200 interactive，30 分钟上限。最初固定 hopper-17，希望控制节点变量；原节点释放后已被其他作业占满，因此在新作业仍排队时取消了节点绑定。后续若换节点成功，不能单独归因于 CUDA 预初始化。

最新检查：r22 尚在排队，原因 `Insufficient amount of resource: ngpus`；没有新的物理结果。输出目录 `outputs/oracle_a1_physical_replay_r22`。本轮未重复提交其他实验，也没有宣称 A1/M2 成功。
