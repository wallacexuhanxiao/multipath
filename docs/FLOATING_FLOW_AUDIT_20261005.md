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


## 合并试验后的结果与 M1 冻结

r48 在 cuRobo 时间参数化处失败，未进入 Anchor。旧成功路径的离线对照没有
复现该失败，不能声称唯一根因已确认。默认 predecessor 的端点处理恢复为
之前的行为。局部开发新增通用 `--predecessor-run`：只有 source、真实初态、
场景、时钟和碰撞检查匹配才复用已有成功 cuRobo 命令，并重新执行 PhysX。
这解决了每次研究 Anchor 都重新规划已通过桥段的问题，符合冻结 M1 的要求。
未通过的物理结果仍明确保存；不通过重置或延长时钟推进。


## r49：启动前导入停滞

r49 / 648479 已分配 GPU，但六分钟内主日志仍为空，CPU 累计约两秒，内存
约149 MB；源码副本已写出，而 `start_simulation` 尚未打印阶段标记。
因此没有进入 PhysX，更没有交互结果。作业已停止，未迁移或继续空等。

确定的启动流程缺陷是：已有 watchdog 只从 Torch/Isaac 的显式初始化开始，
之前的 NumPy/SciPy/相关模块导入没有覆盖。现在将导入 watchdog 移到这些
步骤之前；超时会打印实际 Python 栈，不再出现长期无日志的初始化。
已有节点本地 Torch 打包脚本也扩展为可包含相同安装环境的 NumPy/SciPy
导入资源。原生库仍从原环境加载，遵守节点 /tmp noexec，不换库版本、不改
模拟物理。预打包在 GPU 作业申请前完成。

共享存储上的具体阻塞文件尚未取得运行栈，不能把某个 SciPy 文件说成已
确认的唯一根因。下一次导入应给出明确阶段和超时栈；A1 仍未通过。

## r50：集中修复后的合并物理验证

代码 `57a2638`；Hopper 作业 `648514`，interactive/H200，申请 20 分钟，
实际开发运行耗时 259.47 秒。节点本地科学依赖包已准备好再提交；Isaac
自身启动约 33 秒，完整浮空场景约 47 秒就绪。本轮没有再出现 r49 的导入
停滞，但此前具体哪个文件阻塞仍未取得栈，不能补写成已确认的原因。

M1 使用 r45 已通过的 cuRobo 命令，经本轮初态、固定时间和碰撞检查后
重新在 PhysX 执行。初态差异为零，0.9667 秒 predecessor 再次通过。
局部命令预检查通过，随后执行真实 padded Replay、192 个 MPPI 残差候选、
1 个实测抓持相对变换候选及 60 轮小 residual PPO。PPO 为 64×64 隐层、
24,293 参数，253 个 episode，训练与评估约 122.65 秒。

结果：A1 仍失败，没有发布有效 Anchor artifact，没有执行完整 4.8 秒轨迹。
MPC 最远候选在第 52 帧失败；PPO 最佳确定性评估在第 50 帧失败。
失败类别为 core_object_fidelity，不能解释成初始化、mounted IK 或排队问题。

Replay 第 49 帧仍有实测 opposition contact，位置误差 8.14 mm，旋转误差
0.436 rad；旋转项已使组合交互误差超限。诊断回放继续到第 76 帧时仍有
接触，碗抬高 54.93 mm，但位置误差 53.54 mm、旋转误差 0.859 rad，出口
handoff 失败。手腕自身也存在跟踪误差；现有数据不足以将物体偏转唯一归因
于 controller 或抓持滑移。因此明确结论是：已形成接触和抬升，但交互尚未
实现要求的物体运动。修复控制连续性没有自动解决这个交互问题。

真实回放视频已下载并核查：
`outputs/oracle_floating_a1_scientific_bundle_r50/replay/floating_xhand_actual.mp4`，
47 帧、30 fps、1.5667 秒，从实际 PhysX 网格变换渲染；其中继续收集的
tracking failure 不会将失败改为通过。局部 Replay 内没有 state reset。
机器可读数据见 `docs/evidence/oracle_floating_20261005/r50_result_summary.json`。

本轮 21 个已有/针对已发生故障的局部检查通过，未跑全仓或跨样本测试。
以上修复均针对共同控制、入口、时钟、候选和执行接口；跨样本 transfer
尚未实验验证。下一阶段应基于本轮交互证据定位物体偏转和优化失效，继续
保持 M1 冻结，不扩大门槛或用样本特例宣称成功。本轮不再提交新作业。
