# 默认三解 pipeline：多 source 完整执行

## 当前结论

多解已经并入默认 E2E。248 在一次默认入口中完成三条 4.2 s、无 segment
reset 的连续实际 PhysX 轨迹，每个 RELAXED 区间都有三条不同实测路径。
243 修复后也有三条完整 4.3 s 轨迹：前两条保留 r166 实际成功结果，
第三条由 r170 连续复验通过。253 仍是实际 A1 交互失败。
不能将这些结果称为所有 source 已通过。

已收集的模拟视频与实测 trace/commands 位于
`docs/evidence/oracle_multisource_20261008/20231024_248/` 和
`docs/evidence/oracle_multisource_20261008/20231024_243/`。
各目录的 `solutions_stacked.mp4` 按同一 source clock 纵向对照，
`actual_wrist_paths.png` 按 RELAXED 区间展示实际路径。

## 已集成的默认流程

```text
TACO 原始手/物体观测 + 静态 Oracle schedule
  → local padded Anchor synthesis / 已有局部库初始化
  → 实际前驱到达各 core entry
  → 一条连续 Oracle V0
  → 导出 Anchor core 和 bridge
  → fresh 完整 V0 复验
  → 每个 RELAXED 区间三解（原路径计作第一解）
  → 两条替代完整轨迹的连续 PhysX 验证
  → 本地自动收集，同步三视频和实测路径图
```

`scripts/run_end_to_end.py`、`hpc/run_end_to_end.pbs` 默认每区间三解。
主程序直接调用多解 driver，不需要手工在 V0 后再拼命令。
`scripts/collect_end_to_end.py` 同时收集 source/actual 对照与三解对照。
只有三条完整实际执行全部通过才是 pipeline success。
局部 bank 三解、仅 V0 成功或只成功两条都不会标成三解成功。

本轮进一步补充：

- 主报告保存请求解数、实际完整解数，并在进入多解前记录其结果路径。
  子程序失败时保留具体物理原因，避免仅显示 process failed。
- batch 入口透传各 case 的局部库/源数据与三解请求，报告逐 source 的
  V0 和多解状态。
- collector 可以导出已实际成功的 V0/完整替代轨迹，即使总三解任务
  未完成；结果索引仍记录 `success=false`，不把部分成功冒充完整三解。

## 本轮输入与资源

Hopper `653583.hopper-m-02`，H200 / interactive / 20 min，同一 allocation
内三个 source 独立并行，输出、物理世界、排除区域分别隔离。

| source | 帧数 / fps | 源时长 | 本轮角色 |
|---|---|---:|---|
| 20231024_243 | 129 / 30 | 4.300 s | 新的三解验证；之前只有一条成功 V0 |
| 20231024_253 | 144 / 30 | 4.800 s | 原始 source 的默认 pipeline 对照 |
| 20231024_258 | 128 / 30 | 4.267 s | 已知 carry boundary 失败的迁移检查 |

输入清单：`configs/oracle/taco_put_in_multisource.json`。调度复用已有人工
Oracle，不运行 VLM 或自动推断交互窗口。复用的是已物理验证的局部
Anchor 初始化与同 source 的首段规划命令，不提供完整 EgoEngine
机器人 reference，不设置中间物理状态。每次仍从真实任务初态执行。
这不是没有局部库的冷启动训练实验。

本轮全体使用同样的 planner/controller/优化设置。数据文件、初始化
artifact 和 source schedule 不同；没有 source 名称控制分支或单样本
控制参数。开发候选预算统一为三次，源时间、交互角度和任务条件不变。

## 运行入口

在 Hopper 的 repository 中：

```bash
qsub -v MULTIPATH_CASES=configs/oracle/taco_put_in_multisource.json,MULTIPATH_RUN_NAME=oracle_e2e_multisource_r163,MULTIPATH_MAX_ATTEMPTS=3,MULTIPATH_SOLUTIONS_PER_INTERVAL=3 hpc/run_end_to_end_batch.pbs
```

主作业报告：
`outputs/oracle_e2e_multisource_r163/batch_result.json`。
每 source 保存 `end_to_end_result.json`，多解结果在
`diversity/diversity_result.json`。实际成功以最终物理报告、完整 trace
和 task state 为准。

## 实际结果

### r163：暴露的失败与修复

三个 source 均未完成三解：

- `253`：旧首段缓存的 `scene_id` 与当前场景不一致，主流程直接退出。
  其 source provenance 相同，数值初态差很小，但仍没有撤销 compatibility
  条件。修复为记录缓存被拒绝的原因，用当前场景重新调用同一 cuRobo
  backend，并重新实际验证。不是把旧缓存强行当成可执行。
- `243`：七段均执行成功，但最后任务物体旋转误差 0.37403 rad，超过
  原合同 0.35 rad。主流程只查失败 segment，因此报 No repairable segment。
  现在将这种真实终态失败交给最后一个 release Anchor；其原有 post-context
  若覆盖任务终点，就检查同一个实际终态合同，并提供 object-pose reward。
  保持原时长、原阈值，不额外等待。这不是为 source 243 调阈值。
- `258`：持物桥 `[59,68)` 实际位置已到 1.92 mm，但旋转 0.58230 rad，
  仍超过 30°。反馈模型自己预测 0.56642 rad，说明在当前 Anchor 出口下，
  一再微调下一边界不能证明可达。保持明确失败；本轮没有声称它已修复。

### r164：默认三解入口的复验

Hopper `653587.hopper-m-02`，H200 interactive 20 min。
`configs/oracle/taco_put_in_multisource_retry.json` 运行 `243`、`253` 和 `248`，
各自依旧从原始 TACO 输入和已有局部初始化资源开始，默认每区间三解。
`253`、`243` 检验上述已实际发生的失败修复；`248` 检验多解已经并入
默认入口，不需要上一轮那种手工多解续跑参数。

r164 已完成，三个默认入口均自动尝试三解，但没有完成三解验收：

| source | 完整成功轨迹 | 实际时长 | resets | 结果 |
|---|---:|---:|---:|---|
| 243 | 1 | 4.300 s | 0 | V0 task success；去程、终段缺替代解 |
| 248 | 2 | 4.200 s | 0 | 两条完整 task success；终段缺第三解 |
| 253 | 0 | — | 0 | 重新规划后的 A1 在帧 51 物体交互误差失败 |

243 最终误差 6.27 mm / 4.58°；248 V0 最终误差 10.62 mm / 13.10°。
248 第二完整解相对 V0 的四段实测最大最近路径距离分别为
18.06、22.38、19.30、10.86 mm。
成功视频已下载到 `outputs/oracle_multisource_20261008/`，source/actual
对照与两解纵向同步视频均已解码检查。

### r165 定位与机制修复

作业 `653761.hopper-m-02`，两个 source 同时开发，保留原 source budget。
仍为 243 一条、248 两条；此轮不是三解成功。

1. **Angular handoff history**：243 去程保留旧命令末段的全部姿态，
   从实际入口角度出发时该锁定历史不可达。现在先尝试原历史；若无解，
   在相同角速度、角加速度、首尾位置/速度与时长下重拟合内部姿态。
   位置交接历史和手指命令继续保留，随后实际执行 Anchor 决定接受。
2. **Boundary protection**：短回撤的整条路径可能落在固定保护半径或
   保守 turnaround sphere 内，连缩小障碍也无法生成非空排除。
   保护范围现在按各条旧路径自己的长度与内部空间截断；尺寸搜索仍
   保证每条旧实测路径都被排除，不能用空障碍充数。
3. **Fixed-time sizing**：在原规模无法拟合时自动尝试更小的非空包络，
   保存提出规模、采用规模和拒绝原因；不改变 velocity/acceleration
   budget、物理碰撞与时间。原先两个终段失败输入已在本地重新得到
   nominal 可行解；这只是数值诊断，不算实际物理成功。
4. **Canonical impulse gate**：243 去程的一个候选已经完成 A1，盘子
   线速度仅 0.000051 m/s、角速度 0.000456 rad/s，但旧 baseline 盘子
   角速度为 0.40356 rad/s，因“不匹配旧瞬态”被拒绝。
   保留环境姿态、contact、物体交互条件，将旧速度差记为 diagnostic。
   实测速度原样传给下一段；后续碰撞、连续执行及最终 task 仍须通过。
5. **Measured state vs drive target**：旧首段缓存失效后，下一 Anchor
   的实测 entry pose 被误当作 drive pose。已有物理 Anchor 现在提供
   与 entry 配对的真实 drive 作为规划目标；如果 free entry 的 drive
   已匹配，不因场景缓存不兼容就额外扭曲已学得的命令。它仍作为
   新场景候选逐步实际验证，不撤销 scene compatibility，也不赋值状态。
6. **Precision**：cuRobo 几何转到 float64，并恢复精确边界坐标后再
   做严格 LP 拟合，避免 float32 截断污染固定边界约束。

所有改动都由物理边界、接触模式、source budget、路径几何或真实失败
触发，没有 sequence 名称控制分支。`--resume-diversity-bank` 只复用
成功局部候选；仍要 fresh V0 和完整替代轨迹的连续验证。

### r166

输入 `configs/oracle/taco_put_in_multisource_repaired.json`：243、248、253。
作业 `653804.hopper-m-02`，三个 source 同 allocation 独立运行，
总墙钟 477.13 s。结果：

| source | 每区间局部解数 | 完整成功轨迹 | 原时长 = 实际时长 | resets | 状态 |
|---|---|---:|---:|---:|---|
| 248 | 3 / 3 / 3 / 3 | **3** | **4.200 s** | **0** | 默认 E2E 全部通过 |
| 243 | 3 / 3 / 3 / 3 | 2 | 4.300 s | 0 | 第三条 A3 出口失败 |
| 253 | 未到 diversity | 0 | 未完整执行 | 0 | A1 在帧 53 交互失败 |

248 的三条最终位置误差 10.62 / 23.46 / 18.41 mm，旋转误差
13.10 / 4.52 / 3.22°，全部实际 task success。
四个 RELAXED 区间都在三条完整轨迹中具有实际不同的路径：

| 区间 | 解 2 vs 1 | 解 3 vs 1 | 解 3 vs 2 |
|---|---:|---:|---:|
| [0,26) | 18.06 mm | 22.65 mm | 21.72 mm |
| [65,74) | 22.38 mm | 11.84 mm | 11.87 mm |
| [89,96) | 19.57 mm | 12.28 mm | 31.55 mm |
| [120,126) | 11.20 mm | 16.20 mm | 6.88 mm |

表中为实测最大最近路径距离。不是换 source 充数，也没有渲染放大
运动。短区间的差异更小；本轮没有将它描述成 18 cm 的大弧线。
两段持物 bridge 的 finger drive 最大变化为 0 rad。

243 的第二完整解在去程与原路径相距 99.32 mm，中间持物段
33.16 / 14.35 mm，终段 4.66 mm。第三条执行到 A3 末端时，碗相对
原 canonical 出口旋转误差 0.52959 rad，原条件拒绝；没有放宽 30°
或把该轨迹标为成功。它需要从自己的真实入口重新物理验证 release。

253 的 drive/actual 接口修正已经使用，但重新规划后 drive 仍未完全
匹配旧 grasp 库，A1 Replay/MPPI/PPO 未通过。接口修正不等于抓握已修复。

### r167：将真实 Anchor 失败接入自动修复

多解 driver 现在遇到实际 ANCHOR 失败时，会调用已有 local padded
Replay → MPPI → 小 residual PPO，接着 fresh padded/core 验证，再从
初态完整执行。它消费 runtime 的实测 entry 与已执行 core，不要求
先生成另一个完整机器人 reference。

前驱重放同时保留原 cuRobo dynamics feedforward，避免只复用位置命令
却丢失执行接口。已完成的完整解可以复用实际证据；新解仍必须完整
连续 PhysX。只在不同试验之间初始化物理世界，segment 间不 restore。

作业 `653895.hopper-m-02`，H200 interactive 20 min，续跑 243 的第三条。
前两条 r166 的成功轨迹保留。耗时 363.57 s，结果仍为两条完整成功，
第三条不是成功。A3 的 local padded/core 修复都通过，小 PPO 使用
24,293 参数，保留的是第 8 次更新搜索到、随后实际复验的命令；这不是
deterministic mean policy 收敛的证据。fresh 完整执行通过三个 Anchor，
到达帧 123 / 4.100 s 后最后 free RELAXED 被规划碰撞条件拒绝。

用此次实际记录的网格变换重新检查，规划使用的小指 link1 表面进入
原桌面 0.849 mm。此前 release 检查了物体/contact/速度，却没有检查
松手后的手部桌面净空。重复搜索同一不可用入口 24 次无法解决它。
因此没有撤销碰撞条件，也没有补状态；修复发生在上游 release Anchor。

### r168：free-exit 净空与 artifact 缓存一致性

- local synthesis 复用与 planner 相同的实测手部 triangle surfaces，
  在 free CORE 出口检查原桌面相交。这保护的是真实出现的共同失败。
- 若 outgoing free bridge 报实际入口相交，多解 driver 回到 preceding
  release Anchor。用失败实测状态和已保存 collision geometry 计算
  withdrawal proposal，在原 CORE 内平滑调整、制动，随后重新进行
  padded/core 物理验收，再从初态完整执行。偏移是命令，不是状态赋值。
- prefix 缓存只有在 Anchor artifact 的物理 provenance/validation
  仍一致时才复用 core。修复后的 Anchor 不能被旧的已通过命令盖回。
  prefix 仍实际执行，没有恢复中间状态。

`r168 / 653908.hopper-m-02` 的 Mac rsync 相对目录语义造成上传多一层，
实际入口仍是旧脚本。发现后终止；它不计作新修复的方法验证。
改为直接同步具体文件到模块目录，逐一比对五个实际入口的 SHA256。

`r169 / 653916.hopper-m-02` 以真实修复代码运行，局部失败。
bank 107.10 s，Anchor 修复 401.92 s。Replay 的出口速度 ratio 为 1.37188，
超过原规划预算；MPC/PPO 没有通过局部验收，因而没有完整第三条。

对比同一入口的实测出口，旧 withdrawal helper 同时重建了腕姿态，
实际角速度 z 从原轨迹的 -3.58848 rad/s 变成 +4.27728 rad/s。
此外几何反馈把所有 object bounding boxes 都算作必须远离的区域，
虽然实际报告的相交只有 table。修复 proposal 自己引入了新动态问题。

### r170：几何反馈只改变必要的平移

现在从实际 collision report 提取 obstacle roles；本次只对原 table
计算修复 proposal，其他原规划碰撞条件保持启用。对已物理通过的
local commands 只添加平滑平移修正及对应线速度 feedforward，完整
腕姿态、角速度和手指命令保持原值，随后做同样的 padded/core/full
物理验收。这是由实际几何问题触发的可复用机制，没有 source 分支。

作业 `r170 / 653925.hopper-m-02`，H200 interactive 20 min，已完成，
231.80 s。local Replay 的原 post-context 终态仍失败；MPPI 两次迭代
均得到 valid candidate（7.59 / 14.59 s），随后 fresh padded/core
通过。该次没有调用新的 PPO。新几何 proposal 保留原姿态/手指，
后续 MPPI 的 interaction 修复仍可改变 core 命令。

实测 free CORE 出口手部桌面净空 19.90 mm，planning velocity ratio
0.79200；没有碰撞门槛放宽。第一次完整第三解通过全部 Anchor，最后
0.2 s 的旧 detour waypoint 在新出口下不可用，被原规划检查拒绝。
driver 自动从实际 predecessor 重建这一局部 bank；其余区间保留。
下一次从初态完整执行通过，4.300 s、segment resets=0、task=true。

最后三条完整轨迹的物体终态：

| 完整解 | 位置误差 | 旋转误差 | 末帧线速度 | 原 task 条件 |
|---|---:|---:|---:|---|
| 1 | 6.22 mm | 3.72° | 0.000356 m/s | 通过 |
| 2 | 5.30 mm | 6.64° | 0.00000847 m/s | 通过 |
| 3 | 13.47 mm | 19.25° | 0.02438 m/s | 通过 |

仍使用原 50 mm / 0.35 rad / 0.03 m/s / released 条件。第三条接近
原旋转与速度边界，数据如上；没有额外等待它稳定。持物 bridge
的 finger drive 变化在全部三条中均为 0 rad。

三解的实测最大最近路径距离：

| RELAXED 区间 | 解 2 vs 1 | 解 3 vs 1 | 解 3 vs 2 |
|---|---:|---:|---:|
| [0,23) | 99.32 mm | 78.98 mm | 92.47 mm |
| [61,70) | 33.16 mm | 17.86 mm | 26.63 mm |
| [97,105) | 14.35 mm | 17.14 mm | 4.72 mm |
| [123,129) | 4.66 mm | 33.05 mm | 33.84 mm |

末段差异也包含重新物理合成 release 后的实际入口变化，并非完全
相同物理边界下的独立 planner ablation。Oracle object/contact/time
条件不变，完整执行仍验证真实边界与任务结果。

三解同步视频已下载，129 帧全部解码通过，30 fps / 4.300 s。
最新 243 collection/diversity_result、三条实际 trace/commands 和图在
`docs/evidence/oracle_multisource_20261008/20231024_243/`；原 r166
主报告保留为 `r166_end_to_end_result.json`，没有改写历史失败。
新 release artifact 在 `docs/evidence/oracle_multisource_20261008/r170/anchors/`。

## 结果边界

Oracle core 的 source object 目标、语义、接触与原时间保持不变。
通过实际入口进行局部修复时，Anchor 命令不保证逐元素相同。
验证了三个对应完整组合，没有验证全部 3⁴ = 81 个组合。
当前执行 embodiment 为 floating XHand；mounted arm 及实际液体转移
不在本轮验收范围；assembly self-collision 仍未开启。复用已有 local
库，不声称 cold-start 训练通过。
