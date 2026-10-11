# New source expansion: execution repairs

## Latest result: r339

Put-In 258 has **three complete successful continuous PhysX trajectories**.
The newly accepted third rollout passes all three Anchors and four cuRobo
intervals in 128/30 = 4.2666666667 s, with zero intersegment resets. Final
measured task success is true: released object, 23.86 mm position error,
0.32640 rad rotation error and 0.00004066 m/s speed.

The last repair addresses a reusable execution bug: despite an identical
measured entry and controller memory, the executor discarded a physically
validated terminal bridge and generated a different program. Reusing the
validated drive/wrench pair passes the full rollout; the last interval's
measured exclusion margin is exactly the locally validated 3.85281 mm.
No timing, contact, collision, task or exclusion acceptance was waived.

Local presentation: `outputs/new_source_demos_20261011/20231024_258/presentation/showcase_overlay.mp4`.
The rendering uses measured trajectories; RELAXED is played at 0.25× and
Anchors at 1×. This is a single controllable floating XHand experiment.
It does not establish mounted-arm or bimanual success. Pour and both Empty
sources still need local acquisition/lift repair; they are excluded from the
successful-demo count. Current evidence is in `continuation_r333_r339_summary.json`.

## Goal

Add new TACO sources to the actual-state three-solution overlay demo. Existing
successful Put-In demos remain the baseline. A local Anchor success is not a
complete trajectory success.

## Observations from r317

- Put-In `20231024_258`: initial local acquisition Replay completed through
  frame 65. Fresh padded Replay failed at core exit frame 59 with 73.54 mm
  object position error and 0.47990 rad rotation error; native grasp persisted.
- Replay and fresh padded runs had identical drive commands and actual entry
  state. Their feedforward wrench programs differed because the exporter
  recomputed feedforward from already modified acquisition drives.
- Pour `20231024_246`: the first local interaction failed. After refinement,
  core-exit native opposition was absent; boundary error was 55.47 mm and
  0.39039 rad. This is a physical interaction failure, not a successful demo.
- Empty bowl `20231027_149` and Empty cup `20230928_031`: local preparation
  completed in 392.47 s and 377.81 s. Simulator object initialization failed
  because `isaaclab.sim` is unavailable in the Isaac Sim execution environment.

## Generic changes

1. A validated floating Anchor can now retain its executed command and native
   feedforward wrench arrays together. Fresh padded/core validation, physical
   predecessor reproduction, export, and continuous execution consume the
   same pair. Exact saved programs skip acquisition feedback/preload rewrites.
   Reshaped proposals use newly computed feedforward and need fresh physics.
2. Source objects missing from the cache are converted generically from TACO
   centimeter meshes to meter meshes. Native USD authors mass, rigid-body and
   convex-decomposition collision properties without an IsaacLab dependency.
   Existing cached object assets are reused.
3. The shared interaction configuration records CORE interior tracking error
   as a diagnostic. Physical source boundary conditions, contact retention,
   time budgets, collision, and final measured task conditions remain gates.
   MPC/PPO and full execution use the same policy.
4. Local acquisition libraries may skip redundant surface IK for the same
   object geometry. Actual contact success still requires fresh execution.
   The raw initialization fallback regenerates its own surface IK if needed.

The changes do not dispatch on task or sequence names. The two Empty schedules
are explicit manually reviewed oracle inputs. These scenes model container
handling; they do not model liquid or granular content transfer.

## Verification status

Changed Python sources parse and `git diff --check` passes. r318 job `659415.hopper-m-02` uses a 20-minute H200 interactive allocation and
reuses completed local proposals. Its four source inputs are in
`configs/oracle/taco_demo_expansion_replay.json`.

The native control repair is now physically verified on Put-In 258 Anchor 0:
Replay, fresh padded Replay and fresh exported CORE Replay all succeeded. The
Replay and padded command arrays are bitwise identical; their native wrench
arrays are also bitwise identical (both maximum differences are zero). The first continuous candidate then executed A0, its following bridge and A1.
Its next bridge rejected the actual entry velocity above the planning budget.
Reached-entry local synthesis corrected A1 (exported exit velocity ratio
0.80526), after which that bridge also passed. The release Anchor was then
locally repaired and validated.

Native USD conversion also succeeded for the new bowl 194, cup 095 and target
158; both Empty sources reached physical interaction trials. No new complete
successful three-solution demo is claimed here.

Local measured-state evidence:
`outputs/new_source_demos_20261011/r318_258_anchor_0/exported_core/anchor_0_actual_h264.mp4`
(34 decoded frames, 30 fps, 1.133 s; a local CORE, not a full task).

## Final V0 result within this turn

Put-In `20231024_258` completed a fresh artifact-consuming, initial-state full
PhysX rollout: all three Anchors and four cuRobo intervals passed, task success
was measured from simulator state, no intermediate reset occurred, and actual
duration equals source duration (128/30 = 4.2666666667 s). No VLM or complete
robot reference was required.

The locally decoded H264 video contains all 128 frames:
`outputs/new_source_demos_20261011/20231024_258/final/complete_actual_h264.mp4`.

Pour 246 and both Empty sources still failed their first physical grasp
interaction after refinement. They are not successful demos. The three-solution diversity stage completed its first bank search but failed
with `No alternative solution available in every interval`. Only the baseline
is counted as a complete successful rollout; no new three-path overlay is
claimed. The V0 success remains valid. Further interval detour search is left
for the next turn because the user requested this turn finish within ten
minutes.


## Continuation: r319–r326

### Two complete trajectories of the new source

Put-In 258 now has the original V0 and one complete spatial alternative.
r323 solution 1 executed all seven intervals from the actual task initial
state. Both rollouts last 128/30 = 4.2666666667 s, have zero segment resets,
and pass measured final object pose, release and stability checks. The
alternative final object error is 30.95 mm / 0.21572 rad; final object speed
is 0.000104 m/s. Three solutions are not yet claimed.

Its measured path differs from the baseline in every RELAXED interval:

| Source frames | Mean nearest-path distance | Maximum nearest-path distance |
| --- | ---: | ---: |
| 0–25 | 6.41 mm | 17.71 mm |
| 59–68 | 8.42 mm | 14.45 mm |
| 88–96 | 16.79 mm | 25.04 mm |
| 117–128 | 66.67 mm | 84.19 mm |

The final free-hand stopping endpoint is unconstrained by the task, so its
larger separation also includes different stopping positions. Distances
above describe the actual paths; no path is magnified for presentation.
The task models container handling, not transfer of liquid or granular content.

Local presentation: `outputs/new_source_demos_20261011/20231024_258/presentation/showcase_overlay.mp4`.
It overlays two successful actual mesh recordings and measured wrist traces.
Only playback is slowed: RELAXED at 0.25×, Anchors at 1×; the clip is
287 frames / 30 fps = 9.5667 s. The renderer now labels the actual count of
successful runs instead of always claiming three. Videos decode locally,
and the overlay preview was visually inspected.

### Reusable failures and repairs

1. Diversity predecessor reproduction and following-CORE validation had
   independently rebuilt feedforward from saved drive targets. The reproduced
   state no longer matched the validated rollout. Both now replay the exact
   drive/wrench pair, sliced by recorded source frames rather than assuming
   every controller record starts at frame zero. r319 reproduced the baseline
   state at frame 59 with maximum difference zero in wrist pose/twist,
   hand q/dq and tool pose/velocity. A focused real-record check also verified
   cropped-source slicing and rejection of incomplete controller coverage.
2. Every r319 detour candidate for [59,68) was rejected by a conservative
   waypoint sphere proxy (24/24). The detour code now reuses the existing
   full-mesh cover refinement at invalid rotated waypoints and fitted paths.
   Physical geometry remains covered and actual collision still rejects the
   path. r320 found the two missing alternative bridges in two attempts;
   all four local interval banks now contain three physically validated paths.
   A complete rollout remains required; local counts are not task successes.
3. Resume discarded repaired Anchor artifacts and selected the first failed
   full attempt instead of the latest actual attempt. It now retains the
   latest validated complete artifact set and latest executed prefix. The
   repaired release Anchor enabled r323 solution 1 to finish. No simulator
   state is patched; each final attempt begins from the task initial state.
4. Zero-residual boundary shooting round-tripped every quaternion and changed
   floating-point bits. This made an exact-equality check discard native force
   records. Zero residual now returns the original commands exactly and a
   nonzero residual leaves its untouched prefix exact. In r325, the identity
   trial kept the successful grasp and reached the genuine outgoing fixed-time
   feasibility rejection; the earlier r323 identity trial lost the grasp.
5. Nonzero boundary proposals still recomputed the whole controller program,
   removing physically acquired contact preload before the residual began.
   The r326 repair preserves the recorded prefix and force program; at the
   actually reached residual onset, it adds only the modeled response
   difference between new and original commands. Fresh physical interaction,
   outgoing bridge and exported CORE checks remain necessary. r326 retained
   native opposition in the nonzero trials, but none passed the complete
   boundary conditions. It did not produce a third successful rollout.
6. Development restores wrote dynamic velocities to fixed kinematic target
   objects, producing repeated invalid PhysX API errors. Fixed environment
   roles now skip those velocity calls. This preserves their authored fixture
   support; it does not introduce an object attachment or final-runtime reset.

None of these mechanisms branches on a task or source name. The baseline
bridge solver and nominal timing/limit budgets remain fixed.

### New interaction types: measured attribution

The lifting reward is already present (Appendix C.8), alongside object
tracking (C.3) and native thumb/non-thumb contact (C.7). A new lifting loss
was not the missing component. URDF identity between retarget and native
physics was checked. Fingers tracked their joint targets closely while the
wrist and real contact geometry disagreed with nominal surface fits.

For Empty bowl 149, r318's commanded palm went 36.60 mm below the table,
while the actual palm was table-supported (0.44 mm geometric penetration).
The goal had followed rotation produced by one-sided contact before a grasp
existed. Local free-motion mass/inertia feedforward alone (r321) left the
wrist error almost unchanged (37.53 → 37.46 mm) and did not acquire a grasp.
It is retained as a disabled experiment, not promoted as a successful repair.

r324 keeps the stationary-entry grasp orientation relative to the fixed
support during pre-grasp one-sided contact, while following actual tool
translation. Full measured grasp conditioning starts after native opposition.
This shared experiment reduced Empty bowl Replay wrist error to 7.08 mm;
Pour Replay wrist error was 7.35 mm and cup 8.64 mm. Nevertheless, all three
still lacked native thumb opposition at core exit. Replay, local MPPI and
small residual PPO therefore failed the physical Anchor gate. These sources
produce zero complete successful demos in this batch. Improved tracking
is a component result, not proof of successful grasp or cross-task transfer.

Raw attribution: `continuation_summary.json` and the r319/r320 local-bank
reports in this evidence directory. r326 uses H200 interactive job 659580,
20-minute wall time, reusing the two complete successful rollouts. The
remaining physical gate is the third continuous trajectory; broader task
expansion still requires successful acquisition on a different source type.

## 本轮续工：r327–r335

### 已经有的新物理证据

r328 保留已验证的真实控制前缀，通过第一处 0.3 秒携物桥段 `[59,68)`。
首个抓握 Anchor 及桥段都保持 PhysX 拇指与非拇指对握；没有状态重置或时间延长。
旧终点 z 速度从 0.113468 调整为 0.122775 m/s，其余终点速度分量基本不变。
终点位置与手指指令不变。拟合的速度比为 0.83111，加速度比为 1.00000。
这只是可执行命令提案；后续 Anchor 仍必须从实测入口通过物理验收。

r330 使用统一的 30° 标准，连续通过前两个 Anchor。第二个 Anchor 的出口
仍有真实对握，相对平移为 28.90 mm，相对旋转为 23.59°。
下一处 `[88,96)` 桥段因实测入口速度比 1.02920 而不满足严格拟合预算。
不能瞬间修改这个速度；r333 因此尝试通过前一个 Anchor 的物理执行来修正出口。
腕部单独修正会损害握持或物体边界条件，8 个候选全部失败，未生成第三条完整解。

### 本轮修复及其可复用范围

| 修复 | 原因 | 验证范围 |
| --- | --- | --- |
| 多次续跑继承 Anchor | r326/r327 没有自己的成功修复步骤，就丢失父运行已修好的 artifacts，退回原始版本 | 用真实 r326 续跑链恢复到 r323 的完整验证集合；r328 复用了通过的原生抓握 |
| 携物桥段终点速度拟合 | 旧控制终点速度不能从新实测出口在固定预算内达到 | 第一处短桥段已实际通过；下一 Anchor 从实测速度执行 |
| 统一 30° 容忍度 | 多处握持检查仍使用 0.35 rad，约 20°，与项目已采用的 30° 不一致 | r330 第二个 Anchor 实际通过；接触、碰撞、源时间和最终任务检查继续执行 |
| 局部接触几何计算 | 全物体距离场和高密度手部网格重复查询耗时过长 | 同一实测杯子状态，原查询超过 6 分钟未结束；修后 28.32 秒，提案有限且在既有 MPC 半径内 |
| 从真实首次单侧接触生成提案 | 失败结束时，物体已被单侧接触推歪；该姿态的修正方向不适合初次抓取 | 已接入下一批实际搜索；尚未证明新类型抓握成功 |
| 联合腕部与手指的边界 MPC | 单独调整腕部出口会使已有握持发生内部滑移 | r335 第 7 个候选通过真实 padded/core 验证及其后桥段；仍使用原 8 候选预算 |

几何提案保留真实网格表面点，按 1 mm 去重，并在实测最近表面点附近求导；
最近三角形查询分批以限制临时内存。它们只用于初始化 MPC，不能替代原生接触证据。
终点速度拟合只改变命令，不给 simulator 赋状态。手指调整仅发生在 Anchor 内，
后续携物桥段继续冻结手指指令。

这些机制依赖实际状态、物体几何、共享误差标准和 artifact provenance，
没有动作名称或 sample 名称分支，也没有 Pour 专用控制配置。
既有 V0 桥段求解器保持原样，新增拟合仅用于 diversity 携物入口适配。

### 不同类型 source 的反馈

r329 的 Pour bowl 246、Empty bowl 149、Empty cup 031 均未通过首个抓握，
本批次没有新类型的完整成功视频。Replay 中首次单侧接触到失败结束，物体分别
被转动约 29.46°、55.05°、0.93°；均未形成原生拇指对握。
这支持把几何修正的反馈点提前到真实首次接触，而非使用已经推歪的结束状态。
r334 已完成，三种新类型均未通过；后续 r337 使用实际承载的对侧手指，并保留已有真实接触。
保留 Appendix C.3/C.7/C.8 奖励；未加入新任务专用 reward。

### 可交付结果与当前边界

完整成功数量仍为 Put-In 258 的两条，均为 128 帧、4.26667 秒连续 PhysX。
本地叠加视频已保留：
`outputs/new_source_demos_20261011/20231024_258/presentation/showcase_overlay.mp4`。
RELAXED 播放为 0.25×，Anchor 为 1×；视频慢放不改变模拟时间。
三条局部桥段解不等于三条完整任务解，尚未通过的轨迹不混入成功 demo。

本轮机器可读证据：`continuation_r327_r330_r329_summary.json`。
后续 r334、r335 均已完成，使用 interactive H200；按依赖串行分配，源之间在同一分配内并行。

## 最新续工：r335–r339

### 真正推进了什么

r333 腕部单独修正的 8 个候选全部失败。r335 在同样 8 个候选的预算里，
将后半组改成联合腕部速度与真实接触手指 preload 的物理 MPC。
第 7 个候选通过，修复后的交互能继续执行下一处固定时间携物桥段。
手指修正发生在 Anchor 内，RELAXED 携物桥段继续冻结手指指令。
后续 release Anchor 也从真实前驱入口重新合成，通过本地物理验收。

r335 第三条完整尝试已执行全部 128 帧、4.26667 秒，无段间状态重置。
三个 Anchor 和前面三个 RELAXED 区间都通过；最后 `[117,128)` 自由运动
的实测 wrist 曲线违反虚拟排除约束，因此仍不能计为完整成功第三解。
r336 反复重建最后局部桥段后，完整执行仍失败在相同位置。

### 定位到的执行错误与修复

最后自由区间在 bank 中已经物理通过：实测排除余量 3.85281 mm。
其完整运行入口的 wrist pose/twist、hand q/dq、drive targets、controller
wrench memory、object pose/velocity 与本地验证入口的最大差异均为 0。
但 executor 无条件禁止复用最后自由区间，重新规划后命令最大差异变为
0.36509（25 维控制向量的混合单位），实测排除余量变为负值。
这说明成功局部证据对应的控制程序没有被真正执行。

通用修复：取消这个仅因“末段自由运动”触发的强制重新规划。
实际入口与控制记忆匹配、当前区间排除约束仍满足时，复用已验证 drive。
新 bank artifact 同时保存其实际使用的 `controller_wrenches`，连续执行复用
同一 drive/wrench 对；入口不匹配时仍从实际前驱重新规划。
保持源时间、接触、真实碰撞、物体边界、最终任务及实测排除检查。
此修复不依赖 source 名称或动作类别。

r338 因启动时漏传已有 interaction 物理配置，在初态校验即被拒绝，
不属于轨迹实验结果。r339 已恢复既有配置，使用 interactive H200
作业 `660031.hopper-m-02`，申请 8 分钟。bank 复验用时 50.03 秒，完整初态
连续验证用时 52.08 秒，首次完整尝试即成功；源动作本身仍为 4.26667 秒。

四个 RELAXED 区间的实测排除余量分别为 7.49523、2.63595、3.69163、
3.85281 mm；全部源时间预算不变，三个 Anchor 通过，无段间重置。
末段 drive/wrench 与局部验证程序一致，实测排除余量也完全一致。
这给出干净的修复归因：执行了已经验证的控制程序，验收标准保持不变。

### 不同类型 source 的最新实测结果

r337 的三种类型均执行了 32 个 MPC 候选，随后运行 64×64、24,293 参数
的 residual PPO，20 轮、85 个 episode；lifting reward 均已启用。

| 类型 | 新证据 | 仍未通过的条件 |
| --- | --- | --- |
| Pour bowl 246 | 使用真实承载的 pinky 替代名义 mid 来初始化接触修正 | 仍未形成拇指对握；最佳 MPC 出口物体误差 59.26 mm |
| Empty bowl 149 | 几何修正候选在出口形成原生拇指对握 | 出口物体误差 89.48 mm、旋转 0.65325 rad，交接失败 |
| Empty cup 031 | 半量接触修正候选在出口形成原生拇指对握 | 出口物体误差 155.96 mm、旋转 1.65170 rad，交接失败 |

“碰到或夹到物体”与“完成抬升并达到下一交互边界”仍有区别。
这些结果没有证明新类型成功，也没有证明缺少 lifting reward。
下一轮应针对实际抬升与载荷保持修复局部合成，而非继续追加奖励项。

机器可读证据：`continuation_r333_r339_summary.json`。
