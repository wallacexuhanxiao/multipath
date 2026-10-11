# 双手 Anchor 出口与桥段修复（2026-10-11）

**最新状态（r315）：双手 A0 与 `[32,39)` 物理 bridge 已通过；A1 在 source 46 因左拇指实际失去接触终止。完整 V0 未完成。下文按实验时间记录，前面的排队和未完成描述属于历史状态。**

## 目标与当前状态

保持 `oracle → local padded Anchor → exported core → cuRobo RELAXED → continuous PhysX`。
本轮处理 Brush `(brush, eraser, plate)` / `20231103_057` 的双手版本：右手握 eraser，左手握住桌面上的 plate，没有 fixture。源时钟为 110 帧 / 30 fps，完整任务预算 **3.666667 s**。

已确认：A0 在移动载荷上下文下通过 padded rollout `[14,38)`，导出的 core `[20,32)` 也从真实可达入口重新执行通过；r299 右手工具实际抬升 41.422 mm。r306 新 bridge 执行至帧 37，先前的工具/食指碰撞未在这个实际前缀出现，但左手失去抓取。r307 用这条新 bridge 的载荷重新合成左手，contact-directed MPC 的候选通过 fresh padded/core；右手实际抬升 40.989 mm。随后新 bridge 在帧 33.75 触发 PhysX 工具/盘子接触，两手当时仍保持抓取。

尚未完成：完整物理 bridge、后续 A1/A2、完整 **3.666667 s** 任务。r300/r301 的负面实验改动均已撤回。r308/r309 修复 native-contact 反馈遗漏与 command / predicted-state 混用后，真实 bridge 推进到帧 **38**，双手仍保持抓取；工具与左手食指 mesh 碰撞，实际停止于 **1.266667 s**，零 resets。r310 使用更长的 r309 实测反馈，作业 **659005** 已结束（5 min 23 s）：A0 padded/core 再次通过，但新 bridge 候选不满足联合约束，规划阶段拒绝，未执行 bridge。不能把预测无 mesh 重叠描述成完整任务通过。

## 找到的原因与修复

### 1. 局部有效的抓取出口没有给下一次交互留下可连接边界

旧的 quiet-exit 方案只拟合 A0 自身，末端速度较小。后续 `[32,39)` 仅有 0.233333 s，下一交互位置、姿态和碰撞空间共同限制连接。静态 peer 的边界搜索失败只说明这组条件下搜索未成功，不能证明物理不可达。

修复：从已记录的真实抓取建立点之后，联合拟合 A0 的尾段出口与下一 oracle Anchor 的入口，保持两段各自的时间预算与原有物体边界条件，保留手指命令。候选重新执行 padded rollout 与 exported core 后才算有效。局部验收同时记录真实出口到下一边界的 nominal 可达性；这项检查不等于碰撞或物理验证。

| 实测量 | r275 旧出口 | r280 联合出口 |
|---|---:|---:|
| padded / fresh core | 通过 / 通过 | 通过 / 通过 |
| core 内工具上升 | 21.591 mm | 31.938 mm |
| 出口 wrist Y 速度 | 0.070309 m/s | 0.668412 m/s |
| source-time stretching | 无 | 无 |

工具位移与 wrist 位移还受手物相对变换及 wrist 旋转影响，不用单一轴的速度证明不可达。

![A0 实测工具抬升和 wrist 速度对比](evidence/oracle_action_types_20261008/anchor_boundary_repair_20261011/physical_exit_comparison.png)

### 2. 碰撞预测距离被当成实体占据体积

原碰撞导出用 `contactOffset` 膨胀 native convex pieces。这会把预测接触范围当成实体占据范围，使真实出口在规划器中出现 tool/plate 初态碰撞。

修复：保留 native convex decomposition；占据几何使用非负 `restOffset`，把 `contactOffset` 单独记录。当前 rest offset 为零，contact offset 约 1.3625 mm。**模拟器碰撞参数、摩擦、动力学没有改动**。r281 使用与 r280 完全相同的已验证命令和实测出口后，初态碰撞拒绝消失，进入下一入口搜索。

PhysX 的 `contactOffset` 决定提前生成接触的距离，`restOffset` 决定平衡时的表面间距：[官方说明](https://nvidia-omniverse.github.io/PhysX/physx/5.1.2/docs/AdvancedCollisionDetection.html)。这项修复没有取消 RELAXED 中的实际 tool/plate 非预期接触检查。

XHand 的 CollisionAPI 在 Xform 父节点，Mesh 是其子节点。修正了 peer collider 的归属和坐标变换；当前 SDK 仍不能直接导出这些子 Mesh 的 native cooking 结果，使用实际 authored collision mesh 的完整 convex hull，并明确记录 native export 失败。不能把这个 fallback 描述成已完成 native hand cooking。

### 3. 两只手的终点与路径需要按同一时间协调

静止左手的入口搜索仍失败。新边界优化同时选择右手入口和左手沿桌面的平移；冻结两侧 grasp/fingers，左手保持高度与朝向。r282 找到满足原物体边界、速度/加速度预算和终点碰撞约束的候选：左手桌面平移 `[-26.779, +20.235, 0] mm`，约 33.56 mm。

r282 尚未物理执行：随后单独生成左手路径时仍把右手视为静态障碍，重新引入了独立规划限制，时间拟合被拒绝。r283 修复组件规划后，左手路径生成成功，随后右手在 nominal 时间拟合时被拒绝，仍未执行桥段。

修复：每只手的 cuRobo 路径作为协调运动的候选；组件候选阶段不使用另一只正在移动的手与 payload 的静态位置。生成后恢复全部 peer/payload 碰撞，按同一 physics clock 检查两条完整路径。真实几何相交则拒绝；保守 sphere overlap 需要完整 mesh 确认。最终实际执行仍保留原有 native contact、grasp retention 和状态检查。规划中的 pose override 只用于碰撞预测，未赋值给模拟器。

### 4. 同一边界在两个接口中采用了不同的速度换算

r283 的右手拒绝点为 `No fixed-time boundary trajectory in coordinate 3`。边界优化把空间 angular velocity 转到 ZYX 参数时使用 1e-5 s 有限差分，规划执行使用 1e-4 s；反向转换又引入一次有限差分。候选贴着离散加速度包络时，这些差异改变了可行边界。另外 cuRobo float32 原始路径会把重新写入的精确边界再次舍入。

修复：边界选择、桥段预检查和路径拟合共用解析 ZYX angular Jacobian；从参数速度还原 world twist 使用同一 Jacobian；路径在写入边界前转为 float64。保持原限值与真实 **120 Hz** physics clock，不增加等待。

一次最小的 nominal 边界复算验证了速度往返误差为 3.33e-16、0.233333 s / 28 ticks 的极限边界拟合通过。它不包含碰撞或物理执行，不能替代 r284。第一次手工诊断误用了 240 Hz，已排除为验收证据。

r284 在 Hopper 进一步确认：两侧 cuRobo 路径均通过固定时间拟合。恢复完整 peer/payload 几何后，同步检查在 tick 1 发现工具/盘沿相交；没有执行这个不安全候选。因此速度接口修复已有运行验证，桥段物理成功仍未成立。

### 5. 出口瞬时无碰撞，不代表按实测速度接管后的第一步安全

r284 的 tick 0 没有几何相交，tick 1 已相交。单独导出的 native convex pieces 查询给出第一步工具/盘沿约 **2.224 mm** 穿入，分离法向约 `[-0.4064, -0.4403, +0.8006]`。三角面查询的 depth 不是可靠的实体最小分离距离，因此保留原三角几何拒绝，同时用单个 convex piece 的实体接触方向和深度计算修正。

修复：将这个预测碰撞反馈到真实 predecessor 的局部合成。分离量由预测穿入、现有 native contact band 和 0.1 mm fitter margin 相加得到，约 **5.049 mm**，没有手工设置 sample 偏移。出口位置和速度仍和下一入口一起求解；固定旧速度再平移的方案在 CPU nominal 检查中被拒绝，因此未消耗 GPU 去执行它。

新联合拟合在原有 0.233333 s 尾段与 0.233333 s 桥段预算内通过。手指命令、scene state 与 PhysX 参数保留。r285 重新执行 padded/core 后通过，工具在 core 内实际上升 **41.355 mm**。初次 Replay/MPC 候选未通过出口条件，随后 24,293 参数的 residual PPO 在第 5 轮产生的采样控制通过 fresh padded 和 exported core。这里验证的是导出的控制轨迹，没有声称整个随机策略都已成功。桥段尚未执行：联合终点求解在小约束残差上停住。

几何反馈明确标注 `physical_contact_evidence=false` / `collision_physically_executed=false`。它是已保存 cuRobo 候选的预测碰撞，不能伪装成实际接触、实际物理入口或成功任务。

为避免重复消耗时间，当有合法的已抓取桌面支持手可协同时，尝试一个静态 peer seed 后进入联合求解；其他路径仍保留既有 seed budget。

### 6. 可行边界与 source-score 最优边界不应混为一件事

r285 联合求解多次耗尽迭代后仍有小残差。旧实现只验收最后迭代，继续最小化 source score 也可能丢失中间可行解。变量包含米、弧度、速度，旧变量尺度也不一致。

修复：按原时间、速度和物体容差归一化求解变量；任一迭代满足原约束即保留，不继续追逐 reference 最优值。若仍失败，围绕该候选做最小改动的约束恢复，并记录每条时间、物体和碰撞残差。碰撞验收仍要求 0.1 mm clearance；恢复阶段的 0.01 mm 数值余量只增加保守性。r286 冻结已通过的 A0 控制。第一个 seed 已得到可行终点，两侧 cuRobo 路径均完成时间拟合。同步几何检查前 13 ticks 没有实体相交；tick 14 出现手与手、工具与另一只手的相交，未执行桥段。边界残差修复因此已有运行验证。

### 7. 独立可行路径还需要联合调整中段

r286 表明，两个终点同时可行且两条路径各自可行，仍不能保证同步运动没有碰撞。新增联合拟合复用原固定时间 collision fitter，将两只 wrist 放在同一个优化变量中。r287/r288 先冻结初末状态、初末速度和手指命令，只调整中段；r289 起保留真实初态/初速，允许未验证的 canonical 终态一起拟合。每一步预测都会更新另一只手和其 payload；最终仍用完整实际碰撞 mesh 重新确认，实际 PhysX 再验收。未改 source clock、原物体边界或模拟器参数。

### 8. 固定边界的球体重叠需要核对完整实体

r287 联合拟合把最大 sphere overlap 从 34.833 mm 降至 19.105 mm，随后因固定 tick 27 的 4.321 mm overlap 无法继续收敛而退出，没有执行桥段。CPU FCL 对保存的完整碰撞 mesh 做了一次定位：tick 14 确有三对相交，tick 0、1、27、28 没有相交。这是规划几何证据，不是物理成功证据。

旧覆盖细化把所有路径点对着同一个 peer 位置检查。修复让每个覆盖检查样本使用同一时刻的另一只手及 payload；只用经完整 mesh 确认安全的固定配置细化 full-surface spheres，包含独立建模的 attached object。真实相交、source time、动力学和最终完整 mesh 检查都保留。r288 复用同一 A0，固定样本 0、1、27、28 的联合 clearance 分别变为 0.990、0.564、0.206、0.836 mm，全为正值。完整 hand/body surfaces 保留；右/左 hand cover 数量分别为 154 / 597。覆盖误报修复得到运行验证。中段联合拟合仍失败，未执行 bridge。

### 9. 路线与尚未物理验证的 canonical 入口一起求解

只锁定一次 endpoint search 给出的未验证目标，仍可能把联合中段拟合限制在一个不可用候选中。r289 复用现有 fixed-time fitter，允许入口位置和速度与路线一起调整，但两侧物体入口必须仍在原 oracle 条件内；最终只导出一个 canonical 入口，真实 PhysX 接管才决定它是否可达。已有物理 Anchor 的实际出口、起步速度、手指命令、支持手法向/角度条件和时间预算都保留。非线性 fallback 也保留同一组支持坐标约束，避免 fallback 丢失 table-support 条件。

### 10. 把合法自由度放回路线优化，减少重复物理状态读取

r289 在非线性碰撞拟合中耗尽 20 分钟，调度记录 `Exit_status=-29`。没有进入 bridge 物理执行，不能将其解释为路径物理不可达。旧查询在每个假设下重复读取同一实测 grasp transform、native wrist state 和 collider transform。

r290 仅在 PhysX 暂停的规划阶段缓存这个实际 predecessor，退出规划前清理。缓存与实时几何的 clearance 差异必须小于 1 μm，第一侧实测差异为零。新增每次非线性迭代的残差、查询时间和候选保存；一旦满足原碰撞、时间和 source 条件就停止优化，进入真实执行。移除 fixed-entry 变量对应的重复等式行，避免 scipy 消去固定变量后留下 `0=0` 的约束。一次最小 nominal 检查确认真实初始两点、被保留坐标、时间及动力学限制不变；这不构成 PhysX 成功证据。

旧支持手约束还冻结了桌面 yaw。现在保持桌面法向坐标与 tilt，同时允许路线优化选择平面内 yaw。独立 CPU 几何诊断的 yaw 范围由原 120 Hz / 0.233333 s 离散角加速度预算计算：±0.189583 rad。负向 yaw 候选在原 nominal 限值和联合 source 条件内移除了手与手相交，但仍有工具与支持手相交。它只证明该自由度合法且有用，没有证明 bridge 成功；运行时不使用手写 yaw 曲线或 sample 特例。

### 11. 同几何批量距离查询与允许搜索暂时不可行

r290 三次非线性迭代的 clearance 残差都为 −43.697 mm，第三次累计 108.005 s，其中距离查询占 105.069 s。记录候选后主动结束，避免继续耗尽作业。缓存只消除了重复物理状态读取；逐点碰撞体上传和 peer 差分仍是瓶颈。

r291 用同一 cuRobo world field 和 full-surface covers 批量查询所有 physics samples。peer 几何保持在真实 acquired pose，把查询 sphere 转到对应 peer 的 acquired 坐标，直接对两侧 wrist 求导。先核对初点、中点和终点的旧/新距离，差异超过 10 μm 则拒绝。仅变换规划查询，不改变 PhysX state。

非线性优化复用 elastic SCP 的碰撞残差惩罚，搜索中允许临时相交，最终 clearance、完整 mesh、nominal dynamics 和实际物理检查全部保留。原来硬碰撞线性化在碰撞 seed 上可能没有合法方向，三次原地迭代提供了这个失败模式的证据。还移除了额外的 `0.98²` oracle 优化边界，恢复原 `1.0` 联合条件；实际 5 cm / 30° 验收没有放宽。它是否解决物理 bridge，以本轮实际执行结果为准。

r291 的变换提前转为 float32，距离核对差异达到 27.120 μm，被检查拒绝。r292 把 FK / 坐标变换保留为 double，只在 ESDF 查询处转为 float32，最大差异降至 **0.330 μm**。在实际保存的查询中，平均每条路径耗时从 r290 的 **4.913 s** 降至 r292 的 **0.0403 s**，约 **122 倍**；完整表面与源时间保留。这是求解接口效率证据，不是物理成功。

r292 非线性拟合在 9.95 s 内结束；碰撞残差为 −1.046 mm，但 source 条件为 −1.516、nominal dynamics ratio 为 1.01269，故严格拒绝，bridge 未执行。没有用降低物体精度或提高 dynamics 限值接受这个候选。

### 12. 直接参数化合法运动，分别约束两个物体入口

针对 r292 的 LSQ 求解失败，r293 把实际起步两点和支持手 height/tilt 直接固定在参数化中，仅优化其 null-space 内的合法运动，不再用大量等式要求优化器事后保持这些坐标。两侧 oracle 条件分别作为约束，替代取两者 `min` 的不光滑标量。整体缩放目标函数，保留碰撞惩罚与路线目标的原比例，避免巨大初始 QP 步。

不可改变的 nominal dynamics 行先按原 `1.0001` 数值验收检查，再从优化变量系统移除。最终仍完整复算原 velocity/acceleration、两侧 source 条件、collision、手指与 PhysX 检查。一个最小 nominal 检查验证真实初始两点、支持坐标和两个独立入口条件均保留；r293 的实际结果另行记录。

r293 的 nominal ratio 为 1.000000000001、支持坐标误差为零；最终候选仍距原 0.1 mm clearance 余量差 0.096487 mm，source 残差 −9.338e-6，未接受。r294 围绕该候选做同一约束的最小改动恢复，不再拉回旧路线或旧末速。恢复三次迭代后，clearance 为 0.099998 mm、source 残差 −1.566e-9、nominal ratio 为 1.000000000018；总求解 11.11 s。按原数值误差阈值通过后，才进入真实 PhysX。

### 13. 真实失败定位到支持手的动态抓取

r294 在连续执行到源帧 **37 / 110** 时停止，actual duration **1.233333 s**，segment 内及 segment 间没有 reset。原 bridge `[32,39)` 的前 5 个 source frames 已真实执行，完整 bridge 未通过。

第一处失败为左手 native thumb contact 消失，非拇指仍有接触。左侧 hand/object 相对位置误差 8.293 mm、旋转误差 0.04618 rad；右侧仍保留接触，相对位置误差 2.269 mm、旋转误差 0.07404 rad。没有先触发 tool/plate 非预期接触拒绝。不是用目标动画代替实际执行；该次 actual trace 和本地视频保留。

### 14. 支持负载惯性对照未改善，默认改动已撤回

执行模型原先在 `supported_object` 模式排除 payload 质量和惯性，只保留 hand dynamics 与实测接触 preload。为检查这个近似是否造成失败，r295 增加切向及 yaw 惯性项，实际质量、COM、inertia 和桌面法向均来自当前 PhysX 状态。

两次 bridge 的右/左 commands 最大差异均为 **0.0**。r295 payload mass 为 0.35 kg，新增最大切向力 4.045 N、最大 root moment 0.874 Nm。左侧 native thumb contact 更早在帧 **33** 消失，hand/object 相对位置误差 9.586 mm、旋转误差 0.04544 rad；右侧仍有效。这不能证明补惯性是正确修复；当前 grasp 尚没有证明它能承受该刚性负载假设。**已撤回这项默认代码改动**，恢复已验证的执行接口，并保留负结果和同路径对照证据。

### 15. 将失败 successor 作为局部抓取的 post-padding 上下文

r296 保留 r285 右侧 core、原执行接口与真实 initial predecessor，把 r294 的 cuRobo commands 用作 A0 post-padding 的载荷上下文。先执行 Replay，失败才进入既有 contact-directed MPC，再必要时启用 24,293 参数的小 residual PPO，compute budget 240 s。改变的是局部交互合成；最终 RELAXED 仍由 cuRobo 重新生成，手指冻结，不在 RELAXED 训练 RL。

候选必须重新通过 joint padded rollout 和从真实入口执行的 exported core，再进入完整 bridge 验证。上下文是实际失败 predecessor 的已保存计划和实测 core exit，不是 GT mid-air state；没有直接设置后续 Anchor 状态。

r296 实际运行 337.176 s。零残差 Replay 复现帧 37 的左拇指接触丢失；MPC 和接触 Jacobian 候选的 joint fresh Replay 均在帧 31 出现右手 `core_object_fidelity`。小 PPO 完成 27 次更新、112 个 episode，没有通过的候选。最佳确定性策略在帧 32 的 `core_exit_handoff` 终止，实际为左手只有拇指接触、没有非拇指接触；当步 object-tracking reward 为 0.897116，仍在原可行范围内。它没有改善已有抓取，不能将失败解释成缺少 lifting reward（本侧对象明确由桌面支撑）。

### 16. 双手修复复验遗漏了已验证 companion 的控制力程序

找到明确接口错误：首次 padded Replay 复用右手的已验证 commands **及 core wrenches**；左手修复中的 `primary_control` 却只复用 commands，重新计算了右手 wrenches。r296 初始 Replay 与 fresh candidate 0 在相同 `[20,31)` 的右手 pose/finger/twist commands 最大差异为 **0.0**，但三个方向的 force 差异分别为 **3.584、11.456、12.701 N**，moment 最大差异 **0.3774 Nm**。九个 fresh 候选的右手物体终态完全相同，均在帧 31 失败；这些结果没有隔离左手修复的作用。

修复：所有左手 MPC/PPO/fresh Replay 中同样保留完整的已验证 companion core drive program。交接后的 transport 控制仍从实际出口重算。另在 `core_exit_handoff` 的失败报告中保留具体 transition 条件，避免掩盖接触或 object-boundary 的真实失败。没有修改 reward、物理参数、阈值、source time 或 cuRobo 约束。该修复按 compiled controller artifact 工作，不依赖 sample 名称。

同输入验证 r297 / 658851 已完成：fresh candidate 0 的完整 48 个 core ticks，右手 commands 与 wrenches 相对初始已验证程序的最大差异均为 **0.0**。右手执行至帧 32 仍有 native thumb + non-thumb contact；原先帧 31 的右手失败消失，候选失败发生在左手接触。这个修复有实测证据，但没有解决左手的 outgoing-load grasp。

r297 的零残差仍在帧 37 失去左拇指接触；240 s PPO compute budget 完成 22 次更新、91 个 episode，仍无成功候选。初始 left core exit 的实际 wrist twist 为 `[0.0362,-0.0302,-0.0081,-0.9897,-0.0855,-0.6292]`；手指仍有运动。静止 post-padding 的通过不能证明这个动态入口在移动载荷下保持抓取。当前将这些量保留为诊断，不增加新的加速度、torque 或稳态 gate。

证据：[错误控制力对照](evidence/oracle_action_types_20261008/anchor_boundary_repair_20261011/r296_controller_attribution.json)、[修复后的完整 core 对照](evidence/oracle_action_types_20261008/anchor_boundary_repair_20261011/r297_physical_result.json)。

### 17. PPO 在 compute budget 耗尽前未进入设定的低噪声阶段

原 schedule 按 60 次更新从 1.0 退火至 0.25。但 r297 只完成 22 次更新，最后一个 batch 的 noise multiplier 仍为 **0.73305**。它的 actual compute deadline 比 update-count 上限更早生效，原实现没有对此适配。

修复：统一以 `max(update_fraction, compute_budget_fraction)` 驱动原线性噪声 schedule，每个 PPO batch 内保持同一个 distribution，兼容其已保存的 action likelihood。模型大小、learned standard deviation、reward 和 physical acceptance 均不变；不按 task/sample 设置系数。这个修复使短预算物理实验也进入较小扰动的 refinement 阶段，尚不证明能完成 interaction。

r298 / 658857 复用 r297 的所有输入与 240 s PPO budget，仅验证这项退火修复；作业上限仍为 interactive H200 / 20 min。实际完成 22 次更新、91 个 episode，末批 noise multiplier **0.25193**，进入了原来未达到的低噪声阶段；但没有通过 padded repair。这个结果验证了 schedule 行为，**没有验证 interaction 成功**。

### 18. 桌面压力接管与后续移动载荷的需求不一致

r240 retarget 的 nominal opposition finger 是 **ring**，两面 mesh normals 的 dot 为 **−0.82182**。r294 的实际 core exit 却只有 **thumb + index** 接触；两个 active force vectors 的 dot 为 **+6.52358 N²**，没有触发现有 airborne acquisition 使用的对向力判断。当前 table-supported 分支允许同向表面压力触发 grasp-relative takeover。它可以初始化静止稳定动作，但不能作为盘子在后续 bridge 中随手移动的证明。

这是支持接管过早的具体假设，并不把 force dot 定义为新的成功 gate。修复：当之前实际失败 successor 的载荷上下文要求移动时，支持手继续使用已有的对向接触 acquisition proposer，直到原设计的 pinch 有机会建立；静态支撑仍保留原行为。reward 继续是 Appendix C.3+C.7+C.8，binary contact proxy 和真实 load Replay 的验收条件均未改变。

r299 / 658867 从原 local retarget 重新合成左手 Anchor，复用真实 source-zero predecessor 和已验证的右手完整 core controller，停止把旧的桌面压力程序当成新 grasp 初始化。没有恢复 GT 中间状态；所有改动都只作用于命令。小 PPO（24,293 参数）用 45.057 s、4 次更新、15 个 episode 找到通过的 sampled controls；mean policy 没有通过。所选 commands 随后重新通过完整 joint padded Replay 和真实 core-entry 的 exported-core Replay。这是局部物理 Anchor 的证据，不是 policy generalization 或完整 pipeline 的成功。

新连续 bridge 在帧 35.25 因工具与左手 mesh 碰撞失败；两侧仍保持接触，手物相对位置误差分别为 1.370 mm / 5.620 mm。原时间预算、source boundary、实际状态检查仍生效，无 segment 间 resets。该实验同时重建 acquisition 初始化并进行 PPO 修复，不能把全部提升归因于一个 flag。

证据：[实际接触与接管判断](evidence/oracle_action_types_20261008/anchor_boundary_repair_20261011/supported_acquisition_contact_diagnostic.json)、[r299 局部物理结果](evidence/oracle_action_types_20261008/anchor_boundary_repair_20261011/r299_physical_anchor_result.json)。

### 19. 不保留没有物理收益的反力补偿

r300 / 658877 冻结 r299 的左右 Anchor commands 与 core drive program，只把 bridge 的入口支撑反力改成每 physics step 更新。A0 padded/core 再次通过，但左手在帧 32.5 失去所有 native touch，比 r299 更早。反力闭环没有改善实际跟踪，该实验代码已完整撤回；没有调整摩擦、阈值或 source clock。证据：[负面消融](evidence/oracle_action_types_20261008/anchor_boundary_repair_20261011/r300_reaction_ablation.json)。

### 20. 非预期接触的规划余量与占据几何分开

r299 的 nominal route 最近只留约 0.1 mm clearance。占据几何使用 restOffset 是正确的，但这种距离仍在 native contact-generation band 内，不能据此期待实际闭环路线保持同样间距。r301 从两手和所持物体的实际 PhysX contact offsets 自动计算非预期接触的规划余量；rest envelope、实体碰撞验收、抓取验收和 source-time budget 均保持不变。真实入口及其第一步斜率不可移动；规划器只在剩余时间内拟合路线和未验证的 canonical 入口。无法同时满足时拒绝路径。

实际 contact band 为 2.725 mm。搜索最终仍有 −2.268 mm clearance residual，source condition 为 0.1589；没有执行 bridge physics。它说明这次搜索未找到满足统一余量的路线，不能证明原始物理问题不可达。该默认余量要求已撤回；保留复现实验 patch 和 [失败结果](evidence/oracle_action_types_20261008/anchor_boundary_repair_20261011/r301_uniform_margin_result.json)。不继续围绕新造的规划 gate 做阈值试验。

### 21. 用真实执行偏差校正同步碰撞预测

r299 的 bridge 在 source frame 35 的 right wrist 位置跟踪误差为 18.334 mm，left 为 8.992 mm；远大于 nominal clearance。失败点是 left index 第二指节的 authored collision convex rest envelope。此指节的 native cooked mesh export 不可用，保留了明确记录的 conservative authored hull，没有删掉它；碰撞定位属于这个完整模型的实测 pose 相交证据，不声称已内省 active actor shape。

修复：利用同一源序列、同一真实 predecessor 的已执行 commands 和实际 wrist states，在 source clock 上拟合各手的坐标跟踪偏差。cuRobo joint fitting 同时检查 nominal geometry 与 tracking-corrected geometry；source 条件作用于预测的实际 canonical entry，handoff 再检查真实 PhysX state。入口和首步命令保持连续，手指 command 保持原抓取姿态，原速度/加速度 nominal budget 不变。没有状态赋值、没有加入 human relaxed trajectory fidelity、没有改变控制力或物理阈值。

早停后的未观察尾段使用最后观测偏差作为**未验证预测**，不是物理证据。它只能帮助提出下一条路线；最终仍由新的 uninterrupted PhysX 执行判断碰撞、抓取、时钟和 handoff。已有 CLI `bridge-feedback-run` 复用这项机制。最小检查用实际 r299 trace 确认入口预测偏差为零、29 个 physics-clock samples 对齐，且不同 predecessor 会被拒绝。

r302 / 658898 完成约 216 s 的作业，但最终还有 −3.720 mm clearance residual，source condition −9.85e−5；bridge 没有进入物理执行。该首版反馈还同时约束了 command endpoint 与预测 physical endpoint 贴合 source，尽管 command 要主动补偿 tracking lag。r303 纠正这个额外 command fidelity 条件，只把原 C.4 应用于预测实际入口；真实边界验收仍完全相同。路径无碰撞但 boundary 不满足时，现在进入已有 nonlinear boundary refit，而不是在优化前直接拒绝。证据：[首版反馈拟合失败](evidence/oracle_action_types_20261008/anchor_boundary_repair_20261011/r302_tracking_fit_result.json)。

证据：[实测 bridge 跟踪误差](evidence/oracle_action_types_20261008/anchor_boundary_repair_20261011/r299_bridge_tracking.json)。

### 22. 冻结 finger commands 不代表实际 articulation 是刚体

r303 的预测 source condition 已为正（0.1510），nominal dynamics ratio 仍为 1.000，但 clearance residual 与 r302 完全相同（−3.720 mm）。利用导出的全手 collision spheres、桌面盒体和 fitted path 做 CPU 几何归因：右手 nominal / corrected 的桌面间距均大于 17 mm；左手 nominal 最小间距为 +0.4568 mm，corrected 为 **−3.6196245 mm**。加上 0.1 mm planner margin，正好解释残差，差异小于 2 nm。这不是新发生的 PhysX table penetration；两次 bridge 都未执行。

刚体 wrist-error 模型遗漏了接触中的局部 articulation / grasp-transform 变化。此前 r299 的实测指关节变化达 0.02075 rad，入口固定 grasp transform 预测出的 plate Z 与真实 plate Z 相差 3.733 mm；冻结 command 不会消除这些物理变化。

r304 对**预测器**做最小修正：对已声明 table-supported 的支持手，只将实测偏差投影到模型允许的桌面切向平移与法向轴旋转。真实 root/手指/object state 不投影、不赋值；实际全几何、接触和 source 边界验收全部保持。模型还不能精确预测 finger compliance，投影是明确记录的 proposal approximation，只有新的完整物理执行能证明候选是否成立。没有 sample 名称、frame 专用系数或放松物理碰撞 gate。证据：[支持模型归因](evidence/oracle_action_types_20261008/anchor_boundary_repair_20261011/r303_support_model_diagnostic.json)。

r304 / 658936 的初次拟合仍有 −4.316 mm clearance residual，source residual 为 −5.42e−6，未执行 bridge；[物理/拟合结果](evidence/oracle_action_types_20261008/anchor_boundary_repair_20261011/r304_support_prediction_result.json)。

### 23. 碰撞球失败不能代替完整 mesh 的归因

r305 先保留 rejected fit，不把它作为 accepted path。精确检查同时覆盖每个 nominal 与 tracking-corrected physics-clock configuration，结果没有完整 mesh 相交。因此当前 4.316 mm 是 sphere-cover 误拒，而不是这条候选的完整 mesh 预测碰撞。

之前只按四个旧边界配置细化 cover。joint fitting 后的 canonical 入口和中段姿态都可以改变，旧边界清晰不保证新路线的保守 cover 同样清晰。修复：只有在完整 mesh 全段检查通过时，才按这条新路线细化全表面覆盖，再运行相同 source/time/collision fitter。source 条件失败仍修复/拒绝，实体 mesh 相交仍拒绝，PhysX 状态不修改；最多进行一次 cover retry。最终仍须新的实际 rollout。这保护的是变化后的整段路线，不依赖当前 sample。

细化后，cover 最小间距转为 **+66.5765 μm**，预测 source condition 为 **+8.424e−5**，nominal dynamics ratio 为 1.000。原拒绝还剩的是距 0.1 mm 额外 margin 的 33.4235 μm 差额，不是模型穿透。证据：[全路径 cover 结果](evidence/oracle_action_types_20261008/anchor_boundary_repair_20261011/r305_full_path_cover_result.json)。

### 24. 额外 cover margin 不替代真实闭环验收

r306 仍优化原 0.1 mm margin；若优化未完全达到它，只有同时满足以下原有条件才生成物理试验候选：完整 mesh 全段无相交、全部 cover signed clearance 非负、原 source condition、原速度/加速度 budget、初态与首步连续、原 source-clock duration。没有改成某个 sample 的更小 margin，没有扩大 object 或 joint threshold。保持正距离和完整 mesh 双重验证；不会接受实际模型穿透。

候选之后必须接受未修改的实际 PhysX mesh/contact/handoff/task 检查。这是区分 optimization margin 和真实 acceptance，不是宣称几何 clear 就等于物理成功。r306 / 658953 正在运行。

## 实验记录

| 实验 / Hopper job | 修改或冻结项 | 结果 |
|---|---|---|
| r279 / 658571 | 旧 A0；记录候选的时间、物体、碰撞残差 | A0 通过；静态 peer 入口搜索失败 |
| r280 / 658584 | 联合 A0 尾段与下一物体边界 | A0 padded/core 通过；规划初态被 contact-offset 膨胀几何拒绝 |
| r281 / 658596 | 冻结 r280 物理控制；修正 rest/contact 几何语义 | 初态拒绝消失；7 个静态 peer 入口搜索仍失败 |
| r282 / 658609 | 冻结 r280；联合支持手终点 | 联合终点满足约束；单手静态 peer 路径拟合失败，未执行桥段 |
| r283 / 658630 | 冻结 r280；组件候选 + 同步完整几何检查 | 左手路径生成；右手边界速度换算造成 nominal fit 拒绝；未执行桥段 |
| r284 / 658644 | 冻结 r280；共用解析速度换算、精确边界 | 两侧路径时间拟合通过；同步完整几何在 tick 1 拒绝工具/盘沿相交 |
| r285 / 658651 | 将首步实体碰撞反馈并入出口/入口联合拟合 | A0 padded/core 通过，工具上升 41.355 mm；联合终点求解失败，未执行 bridge |
| r286 / 658754 | 冻结 r285 A0；变量尺度归一化、可行即停、约束恢复 | 联合终点和两侧时间拟合通过；前 13 ticks 没有实体相交，tick 14 双手几何相交，未执行 bridge |
| r287 / 658761 | 两条路径联合固定时间拟合 | 固定 tick 27 的粗 sphere overlap 阻止拟合；完整 mesh 复核该步无相交，bridge 未执行 |
| r288 / 658775 | 同步 peer 条件下细化全表面覆盖 | 四个固定配置的 clearance 都转正；中段拟合仍失败，未执行 bridge |
| r289 / 658780 | 保留真实出口；联合拟合单个 canonical 入口与路线 | 非线性优化耗尽 20 min；bridge 未执行 |
| r290 / 658798 | 缓存暂停中的实测几何、释放合法 yaw、移除重复等式、可行即停 | 两侧缓存几何一致；三次原地迭代后主动结束，bridge 未执行 |
| r291 / 658801 | 同几何批量距离/梯度、elastic 碰撞搜索、原 oracle 条件 | 距离核对差 27.120 μm，检查拒绝；bridge 未执行 |
| r292 / 658807 | double 坐标变换；ESDF 处才转 float32 | 距离核对差降至 0.330 μm，查询约快 122 倍；候选仍违反原 source/dynamics，bridge 未执行 |
| r293 / 658809 | 合法运动 null-space、独立两侧 oracle 条件、整体目标缩放 | nominal ratio 转为 1.000；尚有 clearance/source 小残差，未执行 |
| r294 / 658811 | 近可行候选的原约束恢复 | nominal/source/几何通过；首次真实 bridge 执行到帧 37，左拇指失去接触 |
| r295 / 658824 | 桌面负载的切向/yaw 惯性 feedforward，同一 commands | 左侧更早在帧 33 掉接触；这项默认改动已撤回 |
| r296 / 658835 | 失败 bridge 作为真实 outgoing synthesis context；局部 Replay/MPC/小 PPO | 零残差掉接触于 37；27 PPO 更新失败；发现 fresh companion wrenches 未冻结 |
| r297 / 658851 | 相同输入；修复 companion core 控制力复用，记录 handoff 失败项 | 右手 48 core ticks commands/wrenches 差异均为 0；右手失败消失；左手修复仍失败 |
| r298 / 658857 | 相同输入与 compute budget；噪声退火适配实际 budget | 末批 multiplier 0.25193；22 次更新仍无物理成功候选 |
| r299 / 658867 | 保留右手完整 controller；按移动载荷重新合成左手 acquisition | PPO 45.057 s；padded + fresh core 通过；bridge 35.25 实体碰撞，双方仍有 grasp |
| r300 / 658877 | 冻结 r299 Anchor；逐步更新支撑反力 | padded/core 通过；bridge 32.5 丢所有左手 touch，改动撤回 |
| r301 / 658891 | 冻结 r299 Anchor；native contact band 作为独立规划余量 | 未找到满足 2.725 mm 统一余量的路线，bridge 未执行；改动撤回 |
| r302 / 658898 | 冻结 r299 Anchor；相同 predecessor 的实测误差校正碰撞预测 | 拟合失败，−3.720 mm clearance residual；bridge 未执行 |
| r303 / 658930 | 同样实测反馈；source 条件只约束预测实际入口 | source 通过，仍 −3.720 mm；已定位到左手/桌面预测模型 |
| r304 / 658936 | 支持手的反馈预测遵守原桌面运动空间；实际全状态不变 | 仍 −4.316 mm sphere residual；bridge 未执行 |
| r305 / 658943 | rejected fit 做全段完整 mesh 检查；仅误拒才细化全路线 cover | 完整 mesh clear；细化后 cover +66.577 μm，仍被 margin 差额拒绝；未执行 bridge |
| r306 / 658953 | 额外 margin 作为优化目标；正 cover + exact mesh + 原约束 + 真 PhysX | 真 bridge 执行至帧 37；左手失去抓取，右手保持；实际 1.233333 s，零 resets |
| r307 / 658976 | 用 r306 新运动载荷修左手；右手完整控制程序保持冻结 | contact-directed MPC 的第 5 个 fresh padded trial 通过，fresh core 通过；新 bridge 33.75 发生 native tool/plate contact，双方仍抓取 |
| r308 / 658986 | 冻结 r307 A0；native contact 失败也进入实测跟踪预测反馈 | 联合路径拟合通过；执行前预检查错用 predicted physical boundary，bridge 未执行 |
| r309 / 658998 | 命令时间可行性与实际交接目标分开；A0 / path feedback 保持不变 | 原预算实际执行至 38，双手仍抓取；tool/左食指 mesh 碰撞，actual 1.266667 s，零 resets |
| r310 / 659005 | 同一冻结 A0，预测更新为 r309 更长的实际反馈 | 5 min 23 s；A0 padded/core 通过；source/cover 联合拟合未通过，bridge 未执行 |

所有作业使用 interactive H200，r300–r310 上限为 10 min，其余为 20 min。没有修改原来的 5 cm + 30° **联合椭球**物体验收条件，没有增加等待或伸长 source clock，也没有用 GT 中间状态作为物理证据。

### 25. 新 bridge 的实际载荷要回到局部 Anchor 合成

r306 通过原时间/动力学/source 条件、正 sphere cover 和完整 mesh 检查后真正进入 PhysX。它执行至帧 37，原工具/左食指碰撞未在这个前缀出现，但左手 contact retention 失败：相对位置偏差 5.360 mm、旋转偏差 0.06494 rad，右手仍保持抓取。这说明 A0 对旧 r294 outgoing load 有效，不等于对改动后的 bridge 也有效。

r307 采用 r306 实际失败 rollout 中保存的 bridge **命令**作为 synthesis post-padding load；这些命令不是已通过的物理证据。按原 Replay → contact-directed MPC → 必要时小 PPO 顺序修左手，冻结右手成功控制程序。第 5 个 fresh padded trial 通过 `[14,38)`，随后 exported core `[20,32)` 从真实到达入口重新执行通过。本次通过候选来自 contact-directed MPC，**没有启动新 PPO 训练**。右手 CORE 工具实测上升 40.989 mm。

随后重新规划的 bridge 在帧 33.75 被 native tool/plate contact 拒绝；接触力 0.851 N，PhysX 报告正 separation 2.024–2.078 mm，双方当时仍保持 grasp。实际时长 1.125 s，零 segment resets。这不是左手刚才的 padded/core 证明失效，也不是完整 bridge 成功。

### 26. 实测碰撞反馈漏掉了 native-contact 失败

代码只在 segment 存在 `collision_failure`（mesh overlap）时启用实测跟踪预测，漏掉顶层 `native_contact_failure`。因此 PhysX 在正距离下生成的真实接触失败没有进入这条修复分支。

修复为：同一个 RELAXED interval 的 native-contact 失败也可提供 actual-minus-command 跟踪模型；仍核对两侧真实 predecessor、source provenance 和 physically acquired state。若前一 Anchor 重新合成而改变 predecessor，丢弃旧模型，从新实测入口规划。反馈只改变几何预测，不改模拟器状态、接触距离、摩擦、reward 或物理验收条件。未观测的尾段是预测假设，必须由 fresh continuous PhysX 重新验证。r308 冻结 r307 已通过的 A0，只验证这一分支。

### 27. Command feasibility 与预测实际边界再次混用

r308 的联合路径在原 0.233333 s 预算内满足拟合条件，source condition 和几何预测通过。但返回值的 `wrist_xyz_wxyz` 已经加入 measured tracking correction，代表预测的实际交接状态；后续 `bridge_precheck` 把它当作 nominal command，又报 acceleration reachability 不可行，物理仍停在帧 32。

修复：同时导出 command terminal pose/twist 和 predicted physical terminal pose/twist。存在明确 tracking prediction 时，nominal precheck 只检查 command；执行后的 source/contact handoff 仍检查实际 simulator state，且没有取消任何真正的 timing/physics 条件。未使用反馈的原 backend 行为不变。

用 r308 保存的**同一条** `paired_command_fit.npz` 做最小复算：错用预测实际边界时拒绝，使用 command boundary 后通过，预算均为 0.233333 s。证据为 `r308_command_boundary_attribution.json`；这仅证明接口错误被修正，不是物理通过。r309 使用相同冻结 A0、相同 r307 实测反馈重新执行。

### 28. r309 进入真实执行：两手都保留 grasp，但预测尾段仍不准确

r309 的 command precheck 通过，实际 PhysX 从帧 32 连续执行到帧 38。失败是 `tool / peer_left_004`，即工具与左食指第二段的 mesh 碰撞；native tool/plate 提前接触没有在这一前缀触发。失败时右手相对 grasp 偏差 3.368 mm / 0.10474 rad，左手 8.826 mm / 0.12136 rad，双方仍有 thumb + other opposition。完整 bridge `[32,39)` 仍未通过。

旧预测来自只执行到 33.75 的 r307，后半段固定最后观测误差。帧 38 时，右 wrist 平移预测与实测相差 **8.431 mm**，左 wrist 相差 **14.683 mm**；实际手指关节相对入口最大变化分别为 0.02702 / 0.03847 rad。冻结手指命令不等于实际 finger articulation 完全不动。这些数据证明跟踪预测还不充分，不能把碰撞全部归因于常值尾段，也不能把模型预测作为物理证明。

r310 不改 controller、reward、阈值或时间预算；继续冻结通过的 A0，改用 r309 覆盖至帧 38 的实际记录重新预测/规划。其结果仍需要 fresh continuous PhysX。报告证据为 `r309_physical_bridge_result.json`。

### 29. r310 最新结果：更长历史反馈仍未打通 bridge

作业 659005 已结束，wall time 5 min 23 s。A0 padded `[14,38)` 和 fresh exported core `[20,32)` 均再次通过。新 bridge 的联合拟合在原 0.233333 s 预算下未通过：cover clearance residual 为 −4.326 mm，两侧 source-condition residual 分别为 −0.233 / −0.364；这些是预测/规划残差，不是实测穿透或新物理失败。保存候选的完整 mesh 查询没有报告相交，但它仍违反原 source 条件，不能据此接受。

因此本轮在帧 32 结束，实际前缀 1.066667 s，零 segment resets；没有执行新的 bridge。最远的真实 bridge 证据仍是 r309 的帧 38。两个接口修复已经有 r309 的执行验证，但仅靠延长历史跟踪误差反馈没有解决双手 bridge。后续应以真实 physical candidate rollout 校正这一近似模型，继续保留原时间、入口语义和实际碰撞验收。

## 可复用范围与验证边界

实现读取 schedule、实测 grasp transform、时间预算和碰撞几何；没有 sample 名称或 frame 特例。联合出口、碰撞导出语义和同步 peer 预测可以用于其他相同接口的任务。双手桌面平移只在场景明确声明 table-supported、允许 object/table contact、且另一只手已有真实 grasp 时使用。

当前验证只覆盖这一真实双手场景，尚未声称跨样本实测通过。完整 V0 仍要求后续 Anchor 的 padded/core artifacts、真正连续的 predecessor reachability、初始 RELAXED 不依赖 synthesis context，以及完整任务的实际状态成功。

## 本地实际状态视频

`outputs/oracle_action_types_20261008/r280_bimanual/exported_core/floating_xhand_actual.mp4`

已逐帧解码：32 帧、30 fps、1.066667 s。它是通过的 A0 前缀，**不是完整任务视频**；手和物体来自实际 PhysX 记录。

并排、相同 source clock 的对比视频：
`outputs/oracle_action_types_20261008/anchor_boundary_repair_20261011/a0_quiet_vs_lookahead_actual.mp4`。
同样已逐帧解码 32 帧；左侧 r275，右侧 r280。

最新通过的 A0 实际状态视频：
`outputs/oracle_action_types_20261008/r285_bimanual/exported_core/floating_xhand_actual.mp4`。
同样已解码 32 帧、30 fps、1.066667 s；最后一帧已目视检查。

相同 source clock 的 r280 / r285 实测对比：
`outputs/oracle_action_types_20261008/anchor_boundary_repair_20261011/a0_previous_vs_collision_feedback_actual.mp4`。
已解码 32 帧、30 fps；两侧都只是通过的 A0 前缀。

首次包含真实 bridge 执行的本地视频：
`outputs/oracle_action_types_20261008/r294_bimanual/exported_core/floating_xhand_actual.mp4`。
已逐帧解码 37 帧、30 fps、1.233333 s，并检查最后一帧。只到帧 37 的失败前缀，不是完整 3.666667 s 任务视频。

控制接口修复的同源时钟并排视频：
`outputs/oracle_action_types_20261008/anchor_boundary_repair_20261011/companion_drive_repair_actual_comparison.mp4`。
已逐帧解码 17 帧、30 fps、0.566667 s；两侧均显示 source frames 14–30，对应 `[14,31)`。左侧 r296，右侧 r297，全部是实际 PhysX mesh transforms。它比较失败 local repair candidates 的 companion 控制接口，不是任务通过视频。

最新 r299 实际状态视频：
`outputs/oracle_action_types_20261008/r299_bimanual/exported_core/floating_xhand_actual.mp4`。
已逐帧解码 35 帧、30 fps、1.166667 s，最后一帧已目视检查。它包含通过的 A0 和失败 bridge 前缀；连续 PhysX 实际执行到 1.175 s / source 35.25，末个规则渲染采样为 source 35.0。不是完整 3.666667 s 任务视频。

上下堆叠的同源时钟对比：
`outputs/oracle_action_types_20261008/anchor_boundary_repair_20261011/r294_vs_r299_actual_stacked.mp4`。
已逐帧解码 35 帧、30 fps；两侧均为 source 1–35，上面 r294，下面 r299，最后一帧已目视检查。两者都还没有完成全任务。

r306 实际状态视频：
`outputs/oracle_action_types_20261008/r306_bimanual/exported_core/floating_xhand_actual.mp4`。
已逐帧解码 37 帧、30 fps、1.233333 s；它展示左手失去抓取的失败前缀。

r307 实际状态视频：
`outputs/oracle_action_types_20261008/r307_bimanual/exported_core/floating_xhand_actual.mp4`。
已逐帧解码 33 帧、30 fps、1.100000 s，最后一帧已目视检查；实际执行在 source 33.75 / 1.125 s 停止，末个规则视频采样是 source 33.0。它不是完整任务视频。

r307 **通过的局部 padded A0**：
`outputs/oracle_action_types_20261008/r307_bimanual/fresh_padded/candidate_004/floating_xhand_actual.mp4`。
已逐帧解码 24 帧、30 fps、0.800000 s，覆盖 padded `[14,38)` 的实测状态；导出的 core 单独重新物理执行通过。这个局部视频也不代表完整 V0。

r306 / r307 的上下对比：
`outputs/oracle_action_types_20261008/anchor_boundary_repair_20261011/r306_vs_r307_actual_stacked.mp4`。
已逐帧解码 33 帧、30 fps、1.100000 s；统一 source 1–33，上 r306，下 r307，末帧已检查。它对比两条失败前缀，不是通过的多解。

最新 r309 实际状态视频：
`outputs/oracle_action_types_20261008/r309_bimanual/exported_core/floating_xhand_actual.mp4`。
已逐帧解码 38 帧、30 fps、1.266667 s，对应真实执行的 A0 + bridge 失败前缀。双方保持 grasp 到帧 38，不代表已通过碰撞检查或完整任务。

### 30. 用短 PhysX 候选检验 cuRobo bridge 的执行可行性

r310 表明更长的历史误差曲线仍不能替代物理验证。新增通用 `execution/bridge_physics.py`，复用已通过的 padded/core controller program 和既有 cuRobo 路线，冻结位置、速度、手指和 source clock；局部调整的是 external-wrench feedforward。每个候选都重新从 task initial state 物理执行 prefix 和 core，验证 core 后才尝试 bridge。没有从 reference 拼出 entry，也没有在 segment 间恢复 snapshot。

物理候选逐步检查原 native tool/target contact、实际全手与 carried-object mesh、grasp retention 和原 source boundary。历史跟踪误差只初始化控制候选；滤波后的实测 pose/twist error 提出下一次 feedforward，接管淡入使用实际质量、惯量和已有 PD bandwidth。失败候选保留，不改摩擦、controller gains、几何或验收阈值。候选通过后再从 initial 执行同一个完整 controller program，正常 executor 重复实际 gates，随后继续后面的 oracle Anchor。

这是物理 feasibility refinement，不是新的 RELAXED RL。没有 sample 名称、task 名称或 frame 专用调参分支。A0 从已验证 artifact 复用 padded 证据，同时每个新候选重跑 exported core；这轮不声称重新合成并验证全部 Anchors。

复用 r309 路线的最小 precheck：预算 **0.233333 s / 28 physics ticks**；right velocity ratio 0.978596、left 0.640126；两手 nominal acceleration ratio 均约 1.000000；finger commands 全段冻结。语法检查和 diff 检查通过。r311 / **659112** 最初申请 interactive H200 **10 min**，H200 可用 NUS pool 全部占满后，仅在作业仍为 queued 时改为 H100；实际在 hopper-03 执行 **1 min 59 s**。物理搜索预算 180 s、至多 12 个候选，实际只用 **4 个候选 / 34.684 s**。代码提交 `f120789`。

候选 0 准确复现 r309：physics step 24 / source 38 时 tool 与 left index mesh 相交。候选 1 到 step 27；候选 2 到 step 28 时仍相交；候选 3 **完成全部 28 steps**，native contact、实际 mesh、双手 grasp retention 和原 source boundary 全通过。重新从 task initial state 执行同一 controller program 后，bridge 再次通过，到 source 39 / **1.300000 s**，segment 间 **0 reset**。right source-object error 39.825 mm / 0.274147 rad，left 22.788 mm / 0.273089 rad，按原 weighted C.4 验收，两手均通过。保存的左右 wrist/finger commands 与 r309 逐元素完全相同，变动只有 feedforward，构成直接的执行接口归因。

接着 A1 core command fitting 在 coordinate 1 报无解，尚未执行 A1 interaction。**完整 V0 仍未完成**。证据：[r311 物理结果](evidence/oracle_action_types_20261008/anchor_boundary_repair_20261011/r311_physical_bridge_result.json)。实际状态视频 `outputs/oracle_action_types_20261008/r311_bimanual/exported_core/floating_xhand_actual.mp4`：逐帧解码 **39 帧 / 30 fps / 1.3 s**，末帧已检查；这是 initial + A0 + 成功 bridge 的前缀，不是完整任务。上下对比 `outputs/oracle_action_types_20261008/anchor_boundary_repair_20261011/r309_vs_r311_actual_stacked.mp4`：38 帧、统一 source 1–38，上 r309 下 r311；不是多解视频。

### 31. 物理桥段出口还必须允许下一段接管

r311 的真实出口 linear Y velocity 为 **1.006961 m/s**，规划 nominal budget 为 1 m/s。旧 local core fitter 把下一离散步的速度固定为入口实测速度，再同时要求它不超过 nominal budget，因此在 coordinate 1 无解。

修复没有裁剪实际 q/dq，也没有增大 limit：实际入口速度始终保留，下一离散目标速度在原 velocity budget 内选择，并把真实入口速度用于首步 acceleration precheck。这不是等候或 source-time 延长。r311 Y 方向减速只需要 0.835305 m/s²，在原 5 m/s² budget 内。但它的 world angular velocity 转为 ZYX planning coordinates 后，yaw rate 为 **3.161482 rad/s**；一 physics step 降到原 3 rad/s budget 需要 19.377797 rad/s²，超过原 15 rad/s² nominal budget，所以这条完整 Anchor 接管仍不可直接接受。这里的坐标预算不是机器人硬件额定值。

因此把“下一 persistent-grasp Anchor 的 nominal trajectory 可由该真实出口拟合”接入已有短物理候选循环。复用 r311 通过的 bridge feedforward 作为下一条 proposal，再由同一实测 q/dq 驱动修正。该检查保护刚发生的失败，不是新增任务成功 gate；nominal fit 可行仍不代表 core interaction 物理成功。

编译后的 bridge 保存 commands、wrenches、实际 entry、source/window 和物理候选结果，可重复使用；每次最终连续执行仍重新验证，不能用 cached success 替代新 rollout。代码提交 `e7344c7`，r312 / **659126** 为下一次 10 min interactive H100 验证。

r312 实际完成 **1 min 28 s**。以 r311 的通过控制器为基线，候选 0 明确拒绝上述下一段速度接管；候选 1 在 **19.069 s** 搜索后通过 bridge + A1 nominal fitting。再次从初态连续执行，原 source budget、抓持、碰撞、边界均通过，A1 首次真正进入物理执行。它在 source **40 / 1.333333 s** 因 left `core_grasp_retention` 停止，0 segment reset；右手仍抓持工具。左手 finger command 全程没变，但 thumb force 从 6.458 N 变为 0，index force升到 25.6 N；root angular velocity 明显变化。证据：[r312 入口与首次交互](evidence/oracle_action_types_20261008/anchor_boundary_repair_20261011/r312_anchor_entry_result.json)。

### 32. 位置在容忍范围内不等于有稳定保持控制器

r312 左手实际带有 0.7195 m/s 的 wrist Y velocity。旧 `keep_support` 分支只因为 plate 当前 pose 落在整个 core 的 source 范围内，就使用全段缓停的 hold proposal，并取消该段 transport dynamics feedforward。这没有证明当前移动状态可由这个保持程序稳定接管。

修复：只有已有实际编译/执行的 `support_hold_program` 才可复用保持分支；没有这个程序的移动入口走原来的局部 source-conditioned Anchor trajectory fitting 和 supported-load feedforward。实际 q/dq、手指 preload、source deadline 与 C.4 阈值不变，cuRobo 没进入 core。代码提交 `d1efa53`，没有 task/sample/frame 特例。

r313 / **659134** 完成 **1 min 12 s**。复用 r312 的 bridge controller，baseline 再次通过 bridge + 下一段 nominal fit，然后 fresh initial → A0 → bridge → A1 无间断执行。双手 native thumb/non-thumb grasp 保持到 source **41 / 1.366667 s**，0 segment reset；r312 的 frame-40 thumb loss 没有复现。

当前第一处失败变为 left **core_object_fidelity**：实际 plate source error 为 **37.952 mm / 0.445407 rad（25.52°）**。单项小于 5 cm / 30° 不等于通过原 weighted C.4，组合后超界。left wrist 跟踪误差为 5.247 mm；以入口真实 grasp transform 推算的 command-implied object error 为 46.836 mm / 0.363911 rad，它也超出组合范围。说明现在需要局部 interaction synthesis 的物理优化；不能把掉抓修复、LP 可行或单个阈值内当作 Anchor 成功，也不应再扩大容忍范围。该 command-implied 数值仅是归因，不是实际物理验收或新 gate。

证据：[r313 局部交互结果](evidence/oracle_action_types_20261008/anchor_boundary_repair_20261011/r313_local_anchor_result.json)。后续应从这次真实 predecessor 获取 A1 padded synthesis 入口，接已有 Replay → MPC → small residual PPO，再验证 exported core；A1/A2 padded/core artifacts 仍未全部通过。首个双手 oracle Anchor 和第一个运动规划 bridge 已有重复物理证据，**完整 3.666667 s V0 仍未完成**。

最新 actual-state video：`outputs/oracle_action_types_20261008/r313_bimanual/exported_core/floating_xhand_actual.mp4`，已逐帧解码 41 帧 / 30 fps / 1.366667 s，末帧已目视检查；只包含到 frame 41 的实际失败前缀，不作为 terminal task success。

### 33. 用户授权：CORE 内参考偏差改为诊断，继续真实交互

用户看过视频后要求放松参考跟踪标准并继续。代码 `0afb897` 给 `WindowEvaluation` 增加通用 `strict / diagnostic` 策略；双手开发入口默认使用 diagnostic。只放开 CORE 内逐帧 `core_object_fidelity` 的提前停止，源对象位姿、跟踪 reward、5 cm / 30° 的原组合分数继续记录。没有增大数字阈值，没有 source/task/frame 特例。

真实 grasp/contact、物体相对手的保持、入口 source boundary、出口 handoff、已有 command/collision checks 和固定 source clock 继续生效。报告按每个 Anchor、每只手保存超限帧及位置、旋转和组合误差，避免把“允许继续执行”写成“严格参考跟踪通过”。现有 padded/core artifact 完整性要求保持。

只做一个针对本次改动的回归检查：同样的参考偏差在 diagnostic 下可以继续，但掉抓、入口边界和出口 handoff 失败仍拒绝；连同既有 MPC 检查共 7 项通过。语法和 diff 检查通过。r314 / **659260** 提交 interactive H200 / 10 min，复用 r313 的已编译物理 bridge 和 r307 的已验证 A0 artifact；最终仍从真实 initial state 连续执行。

### 34. r314：参考偏差与桌面支撑接触转动需要分开报告

r314 / **659260** 完成，H200 实际作业约 88 s。原 bridge 再次通过，source 41 的参考组合误差只作诊断，实际执行继续到 source **45 / 1.5 s**，段间 0 reset。与 r313 的 bridge 和 A1 commands/wrenches 逐元素相同；改变的是验收策略，没有证明控制行为得到改善。

帧 45 的两手仍具有 native thumb + non-thumb opposition。停止原因是左手相对盘子的旋转变化 **0.355830 rad / 20.39°**，超过固定 0.35 rad 的 grasp-transform gate；相对平移 **46.249 mm**。不能把这一结果写成接触已经丢失。工具/盘子累计实际接触 8 个 physics steps，impulse **0.077129 N·s**。

通用修正：对明确声明为 `surface_supported_object` 的物体，在 diagnostic CORE 中记录相对变换偏差，仍要求真实 opposition contact。strict 模式、空中携带、RELAXED retention 与入口/出口检查保留原要求。不是按 Brush/source 名称放开判断；不是从物体 reference 自动发现新的 Anchor。针对支撑物体转动、空中携带偏差和真实掉抓的既有检查共 **10 passed**，不增加其他验收层。

证据：[r314 机器报告](evidence/oracle_action_types_20261008/anchor_boundary_repair_20261011/r314_relaxed_tracking_result.json)。实际状态视频已逐帧解码 **45 帧 / 30 fps / 1.5 s**，末帧已检查：`outputs/oracle_action_types_20261008/r314_bimanual/exported_core/floating_xhand_actual.mp4`。

### 35. r315：排除判据误报后，首处真实失败是左拇指接触丢失

r315 / **659317**，代码 `dc3643c`，interactive H200 / 10 min，实际作业约 **89 s**。从 initial 连续执行已验证 A0 与 bridge，原 **0.233333 s / 28 ticks** bridge 再次通过，段间 **0 reset**。与 r314 的 bridge 和 A1 commands/wrenches 全部逐元素相同。

实际执行到 source **46 / 1.533333 s**，左侧 `core_grasp_retention` 失败。这次是原生接触事实：左手 thumb_contact=false、other_finger_contact=true，右手两侧 contact 仍有效。工具/盘子累计接触 **12 physics steps**，impulse **0.096533 N·s**；有交互接触不等于已完成整个擦刷 Anchor。

参考跟踪超限累计 8 条，保留为诊断；`full_pipeline_success=false`、`task_success=false`、`padded_artifacts_complete=false`。A1 padded/core 尚未通过，A2 与完整双手任务仍未通过。现有 continuation 仍主要执行 CORE 拟合，不能声称后继 A1 的完整 padded Replay→MPC→PPO 已接通；这正是下一步局部交互修复的入口。

证据：[r315 原生接触与控制程序对照](evidence/oracle_action_types_20261008/anchor_boundary_repair_20261011/r315_native_contact_result.json)。实际状态视频已逐帧解码 **46 帧 / 30 fps / 1.533333 s**，末帧已检查：`outputs/oracle_action_types_20261008/r315_bimanual/exported_core/floating_xhand_actual.mp4`。短前缀不作为完整任务视频。
