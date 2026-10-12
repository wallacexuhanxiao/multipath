# 其他类型 TACO source 实验与 258 叠加演示

日期：2026-10-12。Hopper interactive / H200，每个分配申请 20 分钟；同一分配内并行运行独立 source。使用静态 oracle schedule，无 VLM。

## 这轮实际跑了什么

这轮运行了 **5 个不同输入**，覆盖 Brush、Hit、Pour、Empty 两种物体；加上反馈后的重跑，共 9 次端到端实验。每次从实际任务初态执行，失败结果保留，未把局部通过计作完整成功。

| 类型 | TACO source | 已获得的物理结果 | 完整 source / 三解 |
| --- | --- | --- | --- |
| Brush / eraser / plate | `20231103_057` | 初始 cuRobo 和抓取 core 通过；出口修复后，padded Replay 与 exported core 也通过 | 未通过，下一段 `[32,39)` 的 0.233 s 桥段尚不可达 |
| Hit / hammer / box | `20231102_003` | 初始 cuRobo 通过；抓取 Replay、MPC、PPO 已实际运行 | 未通过，首个抓取 Anchor 失败 |
| Pour / bowl / plate | `20231024_246` | 初始 cuRobo 通过；新抓取模板及后续 MPC/PPO 已实际运行 | 未通过，首个抓取 Anchor 失败 |
| Empty / bowl / plate | `20231027_149` | 修正自由入口后 cuRobo 通过；部分候选形成短暂双指接触 | 未通过，接触未保持到 Anchor 出口 |
| Empty / cup / plate | `20230928_031` | 初始 cuRobo 通过；部分候选形成短暂双指接触 | 未通过，接触未保持到 Anchor 出口 |

本轮其他类型新增完整成功数：**0**。因此没有把这些输入加入成功多解 montage。Replay/MPC/PPO 失败后仍保留失败口径，没有延长 source 时间、修改 oracle 窗口或直接设置物体状态。

## 找到并修改的问题

### 1. 抬升触发混用了接触力方向代理

原实现要求拇指与其他手指的净接触力方向相反，才开始物体相对运动提案。净接触力方向并不等同于原有 C.7 的“拇指 AND 其他手指接触”条件；这使部分实测双指接触候选始终停留在 acquisition 阶段。

改为原有 native C.7 接触持续一个 source frame 后启动提案。它只控制何时提出抬升，实际 lift、grasp retention、source boundary 与 fresh Replay 仍决定是否通过。修复后 Empty bowl 的候选确实在 frame 25.25 启动；但随后丢失接触，完整 grasp 仍失败。因此不能把它描述成已解决抬升。

### 2. 静止物体 grasp 与迁移模板的命令坐标混用

raw surface retarget 的手腕提案是在 stationary entry tool 中拟合的；迁移模板已经包含新 source 的物体运动。之前用 raw proposal 的 stationary 标记解释模板，导致未夹稳时没有正确去除这段物体运动。

模板 acquisition 现在明确使用 moving source tool 语义，并保存反馈修改前的完整 `nominal_commands`。local Replay/MPC/PPO 继续优化这份模板初始化；此前 local fallback 会丢掉模板，重新回到 raw surface retarget。

最后一轮确认 runtime 与 local refinement 都消费了修正后的语义和完整模板。Pour 的物理抓取仍失败；此修复恢复了初始化的一致性，尚不足以证明该模板能迁移成功。

### 3. 局部 grasp 成功不保证能进入短时桥段

Brush 的旧 core 出口带有不适合下一段桥的动量。旧流程在 planner 尚未导出命令时停止；重复同一局部 Replay 也不会改变这个出口。

新增通用的实际 grasp 尾段提案：从连续物理记录中取已获得抓取的状态，联合拟合 Anchor 出口与下一 oracle 入口，保留手指命令和两个原始时间预算。物体/目标与桌面间隙用于提案；fresh padded/core physics 继续验收。

物理结果：Brush 新 padded window `[14,38)` 和 exported core `[20,32)` 都通过。连续重跑精确复现了该 core 的入口、命令、wrench 和出口，随后仍在桥段的可达性检查失败。实测出口角速度为 `[-1.7757,-0.2792,-0.4878] rad/s`。说明剩余问题是 **实际出口动量与下一个 source boundary 的兼容性**，不是 controller program 在回放时被换掉。

这些修复均无 action/source 名称分支。实验 JSON 只选择数据、oracle schedule、已有物理 artifact；没有新建 Pour 专属 reward、controller 或验收阈值。

## 258：已完成同一场景三解叠加

`20231024_258` 的三条成功轨迹均重新核验：每条实际 PhysX duration 为 **4.266666667 s**，全程无 segment reset。叠加视频将三个独立物理 rollout 放在同一场景，以青/紫/黄区分；RELAXED 段播放速度为 0.25×，Anchor 为 1×。实测 wrist 轨迹同步显示。慢放只影响展示。

[258 同场景叠加视频](</Users/apple/Desktop/semantic verifier/multipath/outputs/new_source_demos_20261011/20231024_258/presentation/showcase_overlay.mp4>)

成片为 1920×1080、30 fps，287 帧全部解码通过，并检查了实际画面。展示 duration 为 9.5667 s。

## 可检查证据

- [首轮和反馈重跑的机器报告](other_source_trials.json)：r340、r341、r342。
- [最后修复实验与控制程序逐项比较](final_repair_trials.json)：r343。
- [本地实际状态诊断目录](</Users/apple/Desktop/semantic verifier/multipath/outputs/other_source_trials_20261012>)：失败视频标为 `INCOMPLETE PHYSX DIAGNOSTIC`；Brush 的局部通过视频独立标记，均不作为完整成功演示。

只做了改动文件语法/JSON 检查、diff 检查、实际 Hopper 回放与媒体解码/画面检查，没有扩展通用测试框架。
