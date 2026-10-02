# Oracle V0 代码审查与修复

审查开始：2026-10-02；收尾：2026-10-03（新加坡时间）。

## 结论

本次沿当前 Oracle V0 的启动、局部拟合、Replay/MPC、artifact、前缀及最终执行链检查并修复了以下问题。修复不包含 sample 名或特殊帧号条件；保留既有 source clock、mounted Franka + XHand、r10 bridge 的规划/执行控制，以及历史多解实现。

代码及定向离线检查已完成。A1 物理成功、完整 4.8 秒 V0 成功仍未获得证据；不能把这次代码审查当成物理验收。

## 修复清单

| 问题 | 修复 | 证据或验证范围 |
|---|---|---|
| 部分启动入口可无限卡在 native 初始化；另一些在报告的 try/finally 之外启动 Isaac | Oracle 执行、Anchor 开发、snapshot acquisition 和诊断共用启动函数，分别记录 Torch import、CUDA probe、Isaac 阶段；180 秒超时打印 Python 栈并退出 | r22 已捕获 CUDA 初始化栈；新共享入口通过语法检查，后续完整作业待运行 |
| PBS batch/diagnostic 的额外 CUDA 探针自身没有超时 | 移除重复探针，交由进程内受限启动函数负责 | 四个相关 PBS 脚本通过 bash 语法检查 |
| `run_oracle_v0.pbs` 仍企图先执行未经局部验收的 core，再把输出当作 validated Anchors | 改为实际 predecessor prefix → local padded synthesis → exported-core validation → 发布 artifact；全部通过后再执行 fresh final rollout | 新增 `--stop-before-anchor`，不执行待合成 core；流程尚待 GPU 验证 |
| MPC 预检查使用旧 drive target，而候选模拟从 measured state 恢复 | 每个 horizon 使用实际 q/dq 作为边界，沿用已有边界插值结构 | 代码路径核对；物理效果待重跑 |
| MPC 位置残差与解析速度残差离散后不一致；预检查没有检查真正下发的速度 | 根据候选离散位置统一计算 feedforward；预检查拒绝不一致指令 | 定向回归证明：静止位置搭配非零 feedforward 被拒绝，匹配时通过 |
| nominal 预检查沿用了 actual velocity 的 5% 容差 | nominal q/dq/ddq 使用 1.0001 数值容差；实际 acceleration 仍为 diagnostic | 原局部拟合保持 .98/.95 预算；未恢复 actual acceleration gate |
| artifact 加载可能接受其他来源、其他控制时钟或非有限指令 | 运行时核对来源、fps、dt、边界时间、contact、指令形状及 finite 值 | 定向回归覆盖错误来源、错误 dt、NaN 指令；既有 padded/core 成功要求保留 |
| 第一个 Anchor 的 synthesis 从 frame 0 开始时，无前驱段导致真实初态也被拒绝 | 允许已记录的实际初态，标记 initial_state；非初始入口仍要求前驱链 | 定向回归通过，未允许 reference-constructed state |
| free contact 边界也被强制要求固定 hand-object transform | 固定手物关系仅对 grasped 模式要求；arm/hand/object/速度/scene/contact 检查仍保留 | 代码核对；没有更改 grasp retention 阈值 |
| schedule 可含重复 Anchor id、非整数帧或非法 fps；局部 fitting 对 physics/source 比例直接取整 | 提前拒绝这些输入及不可整除的时钟组合，避免生成错误长度指令 | 源时钟既有回归通过 |
| finite-state 检查遗漏手指速度、物体 pose/velocity、contact force | 补齐这些非有限状态检查；规划位置含 NaN、空路径或非法时钟也直接拒绝 | Python 语法与既有定向测试通过；未新增 torque/collision gate |
| 失败时最后一次 capture/save 再抛异常，会覆盖原失败、丢失报告 | 分别记录 capture/trace 错误并清除成功标志；尝试保存已取得的实际 trace | 代码核对；不会把录制失败当成功 |
| README 将历史两 Anchor adapter 标为当前 V0，且称 mounted MPC 尚未接入 | 标清当前入口、历史实现和 mounted MPC 已接入的事实 | 文档与实际入口核对 |

## GPU 阻塞与调度纠正

r22 / **643351** 实际再次运行在 hopper-17，GPU UUID 为 `GPU-2a86a7f5-c15a-8370-668f-62ac65860780`。超时栈停在 `torch/cuda/__init__.py:372` 的 `_lazy_init`，调用来自 `torch.ones(..., device='cuda')`；未启动 Isaac。作业 3:34 结束，退出码 1。原始日志在 [r22_stall/run.log](evidence/oracle_startup_20261002/r22_stall/run.log)。

因此故障范围已缩小到 CUDA 初始化；仍不能仅凭这条 Python 栈区分节点驱动、设备状态和容器兼容库问题。没有更改共享节点驱动或重置他人的 GPU。

修复后完整 A1 作业为 **643371**，输出 `outputs/oracle_a1_audit_r23`。最初选择 hopper-35 是错误：该节点属于 AISG_debug，interactive 作业得到 **Can Never Run**。2026-10-03 已将尚未运行的作业改到 interactive 可用且曾成功执行 r20 的 **hopper-24**，保持 H200、30 分钟上限。当前为 Q，原因 `Not Running: Insufficient amount of resource: host`；尚未取得新的 Replay 结果。今后选择节点须同时核对 queue/node_pool，不能只看 GPU 是否空闲。

## 验证与剩余边界

- 定向测试：`tests/test_control_check.py` + `tests/test_oracle_v0.py`，**10 passed**。只新增保护上述具体失效模式的用例，没有扩展测试框架。
- 全部 `src/multipath` 与 `scripts` Python 文件通过 AST 语法检查；四个当前 Oracle PBS 脚本通过 bash 语法检查；`git diff --check` 通过。
- 没有宣称完成全面动态测试或排除所有 bug；当前重点覆盖 Oracle V0 主链，历史 VLM/reference、多解、Cut 路径未重新物理验证。
- 按要求，没有新增 self-collision、penetration、torque 验收体系，没有继续 smoothing 或启动 RL。
- canonical entry 能否从前一 Anchor 经固定时长 bridge 实际到达，仍须连续 rollout 验证；不通过时应调整局部 synthesis/边界，不能靠改状态绕过。
