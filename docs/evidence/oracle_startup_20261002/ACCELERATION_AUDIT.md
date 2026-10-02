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

这是完整 A1 synthesis 作业，不是第七次 startup smoothing diagnostic。结果待回收。
