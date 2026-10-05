# 浮空 XHand 流程集中修复（2026-10-05）

## 为什么改变迭代方式

此前每发现一个控制或接口问题就提交一次 Hopper 试验，导致多轮试验仍重复
执行相同的失败起点。用户要求先集中检查、修复，再统一实验。r47 / 648449
在排队时取消；没有继续训练。此前 r44–r46 的物理结果仍全部失败，不能作为
A1 或完整 V0 成功证据。

## 已集中修复的确定问题

| 环节 | 问题 | 修复 |
|---|---|---|
| 初始手型 | 全零手型把全部预抓取压进短 padding | 使用第一段局部 context 的 free 预抓取手型，经真实 bridge 到入口 |
| 残差接口 | 随机位置残差被额外差分成速度扰动 | MPPI/PPO 保留中心命令的速度；零残差保持同一命令 |
| 手腕控制 | 六维命令中的角速度未参与控制 | 以原阻尼增益跟踪目标角速度，保留力和力矩上限 |
| 入口连续性 | 只对齐 q，未对齐 dq | padding 内匹配实测位置和首步速度，在 core 边界消去修正；不加等待 |
| 抓持候选切换 | 切换时出现 18.6 mm / 0.141 rad 跳变 | 匹配已有 drive target 和速度，过渡占用原有 source 时间 |
| 抓持候选初始化 | 单个 physics step 的偶然接触被当成稳定相对关系 | 以一帧 source 时间内连续实测接触初始化候选；不改变验收要求 |
| MPC 滚动 | 下一 chunk 丢弃已选 lookahead，跳回原始 Replay | 将已评估的 lookahead 保留为下一 chunk 的 prior |
| PPO 初始化 | 丢掉 MPC 的进展，重回最初失败 Replay | 使用得分最高的实测命令候选作初值；入口仍恢复真实 predecessor，不使用失败状态推进 |
| PPO 物理失败 | 终止后再次读取非有限状态可能使训练异常退出 | 物理失败保留为失败终止，终止观察不再读取损坏状态 |
| 抓持桥段 | 冻结实测 finger q 会丢掉握紧 drive target/preload | RELAXED 延续上一段的手指控制目标 |
| 规划边界 | 默认零出口速度不一定兼容下一 Anchor | 可传入真实 canonical 出口 twist，仍在原时间预算内验算 |
| 实测碰撞 | 用初始手型球体检查后续手型 | 实测检查更新手指网格、物体位置及被搬运物体几何 |
| 释放 Anchor | 保持抓持被要求持续到最后一帧，提前释放必失败 | free 出口 Anchor 允许 core 内切换接触；grasped 出口仍保留抓持要求 |
| Padding | 已抓持入口在 pre-context 丢失抓持未立即反映 | 保留已存在的抓持约束；不增加 padding 的人类轨迹相似度要求 |
| Core 导出 | 丢弃 fresh core replay 的实测出口，保存 padded 的旧出口 | 导出 fresh core 的实际出口；core 失败不发布成功 artifact |
| Source 时钟 | 多处 round 可能静默接受不整除时钟或错误窗口长度 | 检查物理与 source 时钟相容，命令长度、入口时间及每步标签一致 |
| 多 Anchor 开发 | 入口只支持 A1，后续容易走回 reference 中间态 | 支持任意 oracle Anchor；非初始入口必须有真实 snapshot、source provenance 和匹配时间 |
| 最终执行 | 默认入口仍只支持 mounted 机械臂 | 增加浮空手分支：canonical Anchors + cuRobo bridges，连续世界中不调用 restore |
| 作业结果 | Kit 正常退出可能掩盖物理失败 | Hopper 各阶段读取实际结果 JSON 决定是否继续；最终要求 full_pipeline_success |

修复集中在共同接口、状态和时序机制，没有样本名称分支。put-in 的 oracle
时间表、原始人手/物体数据和物体资产是实验输入，不能把它们说成可泛化的
控制规则。规划 acceleration budget 仍用于命令可行性；没有新增实测
acceleration 或 torque 验收门槛。mounted r10 bridge、VLM 实验路径和多解代码保留。

## 最少验证与下一轮实验

已有及针对已发现故障的局部检查共 21 项通过，耗时不足 1 秒；同时检查了
Python 语法和 Hopper 脚本。没有跑全仓测试、第二个 sequence 或新 RL 算法。
这些检查只说明接口逻辑一致，不能替代 PhysX。

下一轮只提交一份合并修复后的 A1：真实 initial → cuRobo predecessor →
padded Replay → 必要时 MPPI → 小 residual PPO → fresh padded Replay →
exported core Replay。通过才发布 canonical artifact，随后才能运行依赖后继
和完整 4.8 s 验证。代码已接入最终路径，但完整路径尚无通过结果。

尚需物理结果确认的事项：真实抓持能否保持并驱动物体达到 core 目标；
最终 bridge 能否在固定时间、碰撞模型和手型约束下达到 canonical entry；
连续执行能否复现保存的边界。这些失败不能通过传送物体、重置边界、放宽
reference gate 或拉长 source 时间解决。

装配 self-collision 仍未开启，浮空手结果不证明机械臂可达性。接触证据也
不等于力闭合或承载证明。
