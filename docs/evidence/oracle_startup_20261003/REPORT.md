# 并行推进与首个完整 A1 padded Replay

日期：2026-10-03（新加坡时间）。

## 新物理结果：r23 / 643371

解除单节点绑定后，作业在 hopper-28 运行。CUDA 运算通过，SimulationApp 在约 37.3 秒内完成启动。与 hopper-17 的 CUDA 初始化卡住相比，这是不同节点上的成功；不能据此宣称修好了 hopper-17 的驱动。

**A1 padded Replay 完整执行了 1.766667 秒，与原 synthesis 时间预算完全一致；交互失败。**

- 实际记录：53 帧、30 fps，source frame 30–82；区间从 source frame 29 起执行，未在区间内部重置。
- 首次 core fidelity 失败：source frame 47，物体位置误差 0.00705 m，旋转误差 0.42668 rad。
- 仅 source frame 46–49、62–68 被接触判据标记为 grasped；该判据本身不是持续承重证明。
- core 结束 frame76：contact 为 free，预期 grasped；物体边界误差 0.09438 m / 1.17571 rad。
- 实测 arm 最大速度比 0.9603；实际加速度比仍作诊断，不据此单独否决。
- 没有导出通过验收的 core，M2 仍未成功。

[实际状态视频，1.77 秒](a1_replay_actual.mp4)

视频由真实 PhysX mesh transforms 渲染，没有使用参考动画。已检查首尾画面并验证 53 帧/30 fps。

## MPC 真正执行了物理预测，但下一窗口候选生成有 bug

tick0：12 个候选，2 个通过 nominal precheck 并完成物理预测。
tick24：12 个候选全部被 nominal precheck 拒绝，没有物理预测。

零残差候选的唯一失败原因是手指下限：`right_hand_index_joint1`（合并索引11）最低达到 -0.10845 rad，而下限为0。实测入口 q=.217792，上一条 baseline 指令=.418364；统一位置重基准加上 -.200572 rad 偏移，把原来贴近下限的指令推到了负值。arm 速度/加速度、hand 速度和 feedforward 一致性均通过。这次不能把“无可行候选”解释为任务本身不可解。

**已修复候选生成**：复用 `fit_hand_startup`，从真实 hand q/dq 出发，在原 horizon 和 dt 内将手指候选拟合到现有位置/速度限制内，再走原 precheck 和 PhysX 预测。没有改实际 state、放宽 gate、增加等待或新增 acceleration ramp；不可行拟合仍拒绝。

[原失败候选的 CPU 复现](mpc_hand_projection_cpu.json)：48 步/.4 秒不变，19 处关节越界降到0，完整 precheck 由失败变为通过，hand 速度比 .93209。本轮复现约 .85 秒。**这只证明候选合法，物理交互效果待 r24。**

## 同步工作的具体交付

1. [A2/A3 readiness](../../ANCHOR_READINESS.md)：proposals 已齐，无需重复 retarget；frame76/105 的合法入口尚待实际采集。
2. [一条命令导出结果](../../RESULT_EXPORT.md)：新增 `scripts/export_mounted_result.py`，报告和实际视频共用已有 renderer；初始化失败不伪造视频，物理失败如实报告。
3. GPU 工作改为同一 allocation 内并行：A1 修复验证 + 真实 A2/A3 snapshot acquisition；采集通过后可独立开展 A2/A3 local synthesis。最终连接仍须顺序执行。

下一作业 **644339 / oracle_parallel_r24** 已提交：interactive、H200、30 分钟上限、不绑定单节点。使用现有 batch 脚本，不额外申请多个同时运行的 interactive 作业。

离线检查：10 passed；报告导出已用于本次真实输出。研究目标仍是先获得第一条完整 Oracle V0，不推进多解、VLM 或 RL。
