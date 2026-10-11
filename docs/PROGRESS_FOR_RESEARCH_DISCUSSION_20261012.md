# 与师兄汇报：方法进展与共性问题修复

整理日期：2026-10-11；用于 2026-10-12 汇报。重点是方法与实现发生了什么改变，视频数量只作为补充证据。

## 1. 最初目标与现在的主线

从一条 egocentric demonstration 生成多条任务有效的机器人轨迹：保持交互内容、事件顺序和原视频时间，允许交互之间的空间运动不同。

最大的结构进展是把系统从“完整机器人 reference → 替换部分区间”，改成“Oracle 交互分段 → 仅合成局部 Interaction Anchor → cuRobo 直接生成其余区间 → 连续真实物理执行”。V0 不用 VLM，也不要求先做出整条 EgoEngine robot reference。

每个 Anchor 明确区分 padded synthesis context 和最终 CORE。padding 用于建立、稳定接触；导出的 CORE 必须从物理执行到达的入口重新验证。开发快照必须来自真实 rollout；最终执行不在段间恢复快照或设置物体状态。

**可以作为主要进展讲：我们已得到局部交互与独立规划桥段组合的完整物理证据，接下来处理的是新交互里的真实接触稳定性。** 目前完整结果使用 floating XHand；mounted arm 的桥段证据和 floating hand 的完整任务证据需要分别描述。

## 2. 解决了哪些共性问题

### 2.1 规划路径能算出来，但控制器没有按同一轨迹执行

**原问题：** 规划给出位置和速度，执行接口却主要下发位置；状态恢复与接管也可能保留旧目标。位置接近并不意味着速度连续或下一段能接管。逐帧独立 IK 还会切换解分支，产生关节跳变。

**修复：** 保留规划速度 feedforward，以实际 q/dq 或 wrist pose/twist 作为下一段的边界；统一 drive target 与实际状态的接管语义。mounted IK 使用连续 seed 和完整 Franka + XHand 几何。对已经出现的移动入口问题，下一段拟合检查真实入口速度和首步减速预算。

**证据：** r8→r10 的 cuRobo 几何路径完全相同，同一 1.166667 s 预算内，最大关节交接误差从 **0.312262 rad 降到 0.008457 rad**。这是执行接口的直接归因。双手桥段也在固定同一条路径和手指命令时，通过物理候选修正 feedforward 后完成原 **0.233333 s** 桥段。

**范围：** 桥段成功不代表 interaction 已成功；mounted 桥段成功不代表完整机械臂任务已成功。

### 2.2 时间、边界和局部成功没有被正确接到整条轨迹

**原问题：** 均匀压缩路径可能超过动态预算；移动入口误用 cuRobo 默认时间优化会报错。局部 MPC 预测通过，拼起来却可能失败；释放段仅积分衰减速度还会丢掉原本的退手位置。物理释放成功的出口也可能仍让下一段 planner 无法启动。

**修复：** 在原 source-time 内做时间参数化，不可行就拒绝；移动入口使用适用的 cuRobo 配置。局部优化输出先作为 proposal，再从真实 predecessor 重跑 padded 与 exported CORE。释放段保留退手位置并在原区间减速；出口碰撞反馈只修正命令，重新物理验证，不移动物体。

**证据：** 已有完整 Oracle runtime 从实际初态执行七段，源时长与仿真时长同为 **4.8 s**，段间 **0 reset**；runtime 直接消费局部 artifact 与规划桥段。后继边界修正只使用下一 Anchor 条件，RELAXED 内没有逐帧人类轨迹相似度 gate。

**研究价值：** 时间对齐、真实入口可达和跨段连续性已进入执行链，而不是只在离线拼接的视频里成立。

### 2.3 抓取 retarget 的几何目标与实际接触不一致

**原问题：** 缩放 MANO 指尖接近工具，不等于 XHand 的真实指节面能形成对握。整工具凸包还可能填掉柄部周围的空隙，错误地阻止手指靠近；只看平均 IK 误差会掩盖拇指或某根手指没有接触。接管时把实测 hand-object transform 当成 drive transform，又会撤掉抓握所需的预载。

**修复：** 使用真实工具表面与 XHand 接触面初始化抓取；按完整手几何检查碰撞，以实际 thumb + non-thumb contact 验证对握。准备手型与闭合行程由 URDF 和原时间预算约束。实际状态用于物理证据，drive/tool transform 用于保留已有抓握控制偏置，两者不混用。

**证据与边界：** 这些修复进入共同 Replay/MPC/PPO 路径，已有 Brush 首个双手 grasp Anchor 通过 fresh padded 与 CORE 验证，并有工具实际抬升证据。但“工具动了”“接触过”“IK 误差小”都没有被当作完整交互成功。新动作类型的完整 rollout 尚未通过。

### 2.4 Residual PPO 的执行和 reward 曾偏离预期

**原问题：** acquisition controller 会在线重写 nominal wrist，部分 residual 被覆盖或重复叠加；候选只保留 pose commands、丢掉原外力程序，也会让重放变成另一套控制器。早期 objective 缺少 lifting 项，并混入额外 bridge/terminal penalties。短预测前缀被提交时，下一 chunk 可能已经失败。

**修复：** PPO 对固定局部参考使用 a_ref + residual，保留 nominal velocity feedforward；artifact 保存 commands 与 wrench program。按 EgoEngine Appendix C 区分 object tracking、真实接触 bonus、适用时的 lifting，以及只累计 object tracking 的 evaluation。TACO 不启用 human-mimic、smoothness 或 domain randomization。采用独立 **64×64 actor/critic，约 2.4 万参数**；H=20、两 chunk lookahead 全部可行后才提交当前 chunk。

**证据与边界：** 小型 PPO 已在实际 PhysX 局部合成中运行，曾产生通过 fresh padded/core 的采样控制程序；这不等于确定性 policy 已学会完整任务。论文没有公开的精确系数、网络大小和噪声数值，明确记为工程选择。后续实验也表明“没有 lift”不能全部归因于缺少 lifting loss，几何、接管、负载和场景支撑同样会导致失败。

### 2.5 初始化过慢和碰撞拒绝里，存在真实实现 bug

**原问题：** 完整手表面细化后，上万碰撞球即使没有需要检查的 self-collision pair，初始化仍做平方规模循环。cuRobo 原 forward 内核还把 sphere_index × 4 存入 int16，超过 8192 个球时会溢出。整凸包与接触边距的混用也会误报占据空间。

**修复：** self-collision pair 为空时直接返回；大球集合以宽索引分批送入原内核再合并，保留完整球集合。采用实际 native convex decomposition，区分 restOffset 占据空间与 contactOffset 接触生成距离；保守球误报由完整 mesh 确认，不能直接删掉真实几何。

**证据：** 约 1.4 万和 1.86 万球的 GPU FK 核对中心误差小于 **0.1 µm**，对应桥段真实执行通过，未再出现原非法 CUDA 地址错误。

**范围：** 另一次 CUDA 初始化卡住只定位到了 Torch lazy initialization，不能声称当时已经查清节点驱动根因；它与上面的已确认代码缺陷分开报告。

### 2.6 双手交互不能由两个独立单手成功来证明

**原问题：** Brush 源视频有左手支撑盘子，原场景只复现右手，盘子会被推走。加上左手后，两只手独立规划又会把对方当成静态障碍；独立 Anchor 即使成功，真实出口的速度和负载也可能让下一段掉抓。

**修复：** 添加动态左 XHand，与右手、工具和盘子在同一 PhysX clock 下执行；不以 fixture 代替最终支撑手。协调候选按同一时刻检查 peer hand、payload 与完整 mesh。保留双手 commands/wrenches，基于真实出口做局部物理可行性修正。

**证据：** 双手 A0 padded/core 与其后的固定时间 bridge 已通过。r309→r311 的腕部路线、手指命令逐元素相同，仅控制 feedforward 改动后桥段由碰撞失败变为通过。说明“规划几何可行”与“闭环物理可行”需要分开解决。

### 2.7 把优化预算和真实失败条件混为一谈

**原问题：** planner 的 nominal acceleration=15 被直接用于否决 PhysX 实测 dq 的差分；CORE 内参考位姿偏差、桌面支撑物体的接触转动，也会被描述成掉抓。

**修复：** nominal acceleration 继续约束规划/拟合，实际 acceleration 作为诊断。按用户授权，双手开发模式将 CORE 内 source fidelity 改为诊断；桌面支撑物体允许记录接触转动，仍要求实际拇指与非拇指接触。空中携带、RELAXED retention、入口出口与时间条件继续检查。

**干净的归因：** r313→r314 的整段 commands/wrenches 完全相同，只改验收策略，因而不能宣传为动作改善。r314 停在 frame 45 时两手仍有实际接触，触发的是固定相对变换阈值；进一步修正后，r315 在 **frame 46** 出现左拇指真实失去接触。

**研究价值：** 现在能区分参考重现精度、物理接触有效性和任务完成。放松后的实验不能直接与论文严格 C.4 的 success rate 比较。

## 3. 多解上的方法进展

多解已接入完整 pipeline。每个 RELAXED 区间独立维护旧路径排除区域；程序从上一条实测路径生成障碍，并按距离和原时间预算搜索尺寸，不手工放障碍，不跨区间共用。

已有相同交互条件、相同 source clock 下的空间不同完整轨迹证据。短时间区间允许的差异小，不能要求所有桥段都是大弧线。现有成功覆盖 Put-In 的多个 sequence，不能据此宣称跨动作类型、所有局部解组合或冷启动稳定性已经证明。

## 4. 现在还没有修通的部分

最新双手 Brush A1 从真实 initial → A0 → cuRobo bridge 进入 CORE；r315 在 source frame 46 / **1.533333 s** 终止，左手失去 thumb contact，右手仍抓持工具。已经出现真实 eraser/plate contact，但 A1 交互没有完整通过。

后继 A1 当前仍主要执行 CORE 命令拟合；需要把已有局部 padded Replay → MPC → small residual PPO 完整接到这一真实 predecessor，再验证 exported CORE。A2 与完整 **3.666667 s** 双手 rollout 也未通过。

所有修复按几何、实际状态、接触模式或时间预算运行，没有 source-name 特例。**代码结构可复用不等于已证明所有任务都能迁移。** 目前不把 VLM、真实清洁/流体效果、完整 mounted-arm 任务或真实机器人成功计入成果。

## 5. 可以直接讲给师兄的版本

“这段时间主要进展是把 Interaction Anchor + motion planning 的方法真正接成了物理执行链。我们只合成交互局部，其他区间由 cuRobo 独立生成，并保持原视频时间。过程中修了几类共性问题：规划速度没有进入执行器、真实入口与旧控制目标不一致、抓取几何和实际接触不一致、PPO residual 被覆盖，以及完整碰撞模型导致的初始化和 CUDA 索引问题。已有同一 source clock 下连续、空间不同的完整轨迹证据。现在扩到双手支撑后，抓取和桥段已通过，剩余瓶颈是擦刷 Anchor 中左拇指真实失去接触，还没有完整双手成功。下一步集中解决这一局部交互，再做完整连续验证。”

## 6. 可追溯材料

- [mounted bridge 的同路径对照](/Users/apple/Desktop/semantic verifier/multipath/docs/evidence/oracle_v0_20261002/REPORT.md)
- [完整单手 Oracle runtime](/Users/apple/Desktop/semantic verifier/multipath/docs/ORACLE_V0_RESULT_20261006.md)
- [跨动作类型暴露的几何、初始化与抓取问题](/Users/apple/Desktop/semantic verifier/multipath/docs/ACTION_TYPE_PIPELINE_20261008.md)
- [Appendix PPO 对齐与数值边界](/Users/apple/Desktop/semantic verifier/multipath/docs/PPO_APPENDIX_ALIGNMENT_20261009.md)
- [双手物理桥段及最新 A1 诊断](/Users/apple/Desktop/semantic verifier/multipath/docs/ANCHOR_BOUNDARY_REPAIR_20261011.md)
- [同源三解同步视频](/Users/apple/Desktop/semantic verifier/multipath/docs/evidence/oracle_diversity_20261007/three_solutions_stacked.mp4)

以上历史材料按各轮当时状态记录；最新双手状态以本页和 r315 实测报告为准。
