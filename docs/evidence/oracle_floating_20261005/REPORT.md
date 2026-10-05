# Floating XHand Oracle V0 integration

## Scope

Following the user's embodiment change, Anchor synthesis and RELAXED transport
now use the same floating XHand. The physical arm and mounted IK are absent from
this backend. The old mounted backend and r10 evidence remain available.

- Local source proposals, oracle core/padding intervals and 30 Hz clock are reused.
- Wrist uses the prior reproduction's external-wrench PD gains, clamps, COM-origin
  velocity correction and total-hand gravity feedforward; fingers use physical drives.
- cuRobo sees six virtual planning joints (XYZ translation, ZYX orientation),
  actual hand geometry and, when carrying, measured carried-object geometry.
  Virtual joints are never used to set the simulator state.
- Fixed-time parameterization is reused. Physical state recording and core
  object/contact acceptance remain active. No full robot reference or VLM is used.
- Floating MPPI retains 20-frame chunks and 40-frame lookahead, Cartesian rotation/
  translation/finger residuals, followed by the existing 64x64 PPO if necessary.
- Artifacts and compatibility distinguish floating wrist pose/twist from arm joints.

The local development entry is `hpc/develop_floating_anchor.pbs`. It plans a
physical predecessor to synthesis-start, then tests padded synthesis and exported
core replay. This is not yet the final oracle executor: the final bridge must
reach the acquired canonical core entry, and all three Anchors must execute in
one uninterrupted 4.8-second rollout. No success is inferred from this development
prefix. New floating snapshots are acquired; mounted r10 snapshots are not reused.

## Integration evidence

- r35 / 647724: simulation and virtual cuRobo FK initialized; raw synthesis entry
  collided with the open-hand geometry. Rejected before motion.
- r36 / 647726: generic local vertical entry search found a collision-free target;
  fixed-time cuRobo path planned for 29/30 s. Measured collision-model rejection at
  frame 26: position error 1.84 mm, orientation error 0.145 rad.
- Planning now inflates hand geometry by the measured ready-state position error
  plus the rotational swept radius. This is a generic tracking allowance, not a
  source-specific offset or controller tuning. Measured collision checks use the
  underlying geometry, without double-counting the planning allowance.
- r37 / 647727 exposed the double-counted allowance in the first implementation;
  r38 / 647733 runs the corrected check and enables floating MPC/PPO fallback.

Four focused new checks cover virtual-coordinate roundtrip, spatial angular
feedforward, fixed source sample count / padding boundary, and measured collision
checks without double-counting tracking margin. Six existing control/MPC checks
also pass. These are interface checks, not physical success evidence.

## Validation boundaries

Floating-hand feasibility does not imply feasibility for a real mounted arm.
The planning hand shape is frozen at the measured bridge posture. Hand self
collision is disabled as in the retained simulation; stronger self-collision
validation is not claimed. Planning budgets are explicitly nominal, not hardware
ratings. Continuous full-V0 execution, later Anchor dependencies, and physical
carried-object transport remain to be validated.


## r38 and r39

r38 (647733) completed the entire 29-frame cuRobo predecessor with nominal
measured-geometry collision checks passing. End wrist error was 8.71 mm /
0.168 rad, and contact remained free. It then rejected an added nominal wrist
tracking threshold, rather than an actual canonical boundary violation.
That added gate has been removed: a development target is not a validated
canonical entry. Tracking error remains reported; actual collision, source clock,
contact/object interaction and canonical state compatibility remain checked.
No reference state is assigned. r39 (647735) runs from this actual state into
padded Replay, followed by floating MPPI/PPO as required.

The snapshot provenance now points to the actual local trace and source frame,
with the predecessor trace retained, rather than only naming the earlier bridge.

r39 has now printed `FLOATING_PREDECESSOR_PASSED` and entered actual floating
MPPI. First two 64-candidate iterations took 19.15 s / 45.38 s cumulative, with
no valid plan yet. A1 acceptance remains pending; the small PPO fallback is enabled.
Job is interactive H200, 20-minute allocation. No full V0 success is claimed.

The r38 predecessor actual-state video is exported locally as
`outputs/oracle_floating_a1_r38/predecessor/floating_xhand_actual.mp4`.
It contains 29 frames at 30 fps (0.9667 s), rendered from measured PhysX mesh
transforms. The final rendered frame was visually inspected. It shows the
approach only, not a successful grasp. Initial plus N measured boundaries are
rendered as N source-time intervals to avoid adding a frame of artificial waiting.


## r39 final result and entry-posture attribution

r39 completed in 3m47s (PBS). All 192 MPPI candidates physically executed;
all failed core object fidelity, furthest at frame 52. PPO ran 60 updates /
253 episodes in 109.8 s; best deterministic evaluation failed at frame 51.
Replay failed at frame 49. No physical Anchor or full rollout passed.

Direct old/new local-retarget comparison, after aligning object-world transforms:
max wrist position delta 0.00242 mm, rotation delta 5.32e-6 degrees and finger
joint delta 8.92e-5 rad. The local retarget solution itself is effectively the
same as the earlier reproduction. The source reference is not replaced with the
old full robot trajectory by this comparison.

The new zero/open ready hand reaches frame 35 with thumb joints 0.257 / 0.449
rad against targets 0.763 / 1.109 rad; max finger error 0.720 rad. The previous
reproduction had already acquired thumb posture 0.911 / 1.140 rad there.
Closing an initially open hand inside only six frames of context changed the
physical Anchor entry substantially. r40 (648401) uses the first local context's
pregrasp posture as a free, collision-separated task initial hand configuration;
objects still start on the table. It runs Replay only to attribute this entry
change before further optimization. The bridge physically transports this hand,
and all subsequent states are measured. No extra source time is inserted.


## r40 result and residual-controller correction

r40 Replay still fails at frame 49, so correcting ready-hand posture alone is
not sufficient. Thumb q at core entry improves from 0.257/0.449 (r39) to
0.922/1.134 rad (r40), close to the previous reproduction's 0.911/1.140 rad.
Native thumb AND finger contact begins at frame 47 instead of 49. Other fingers
still show tracking lag; a valid grasp throughout the core is not established.
The actual-state failed local video was rendered and its final frame inspected.

A separate code comparison found a concrete porting error: the old reproduction's
MPC applies sampled wrist pose residuals while preserving nominal wrist velocity
feedforward. The floating adapter differentiated random residual positions and
added those derivatives to feedforward, implicitly perturbing a second control
channel. `apply_residual` now retains baseline twist in both MPPI and PPO.
RELAXED planner velocity feedforward is unchanged. A focused regression confirms
nonzero pose residuals change pose commands without changing velocity feedforward.

r41 / 648405 uses the corrected pose-only residual interface with the r40 physical
initial posture. No reward, tracking threshold, controller gain, force clamp,
physics material or source timing was changed. Results pending.


## r41–r43: physical interaction evidence, not another startup gate

r41 (648405) completed in 264.7 s. The corrected pose-only residual interface
alone did not solve A1: all 192 MPPI candidates failed (191 object fidelity,
1 core-entry contact); small PPO completed 60 updates / 253 episodes, with its
best deterministic evaluation failing at frame 51. No accepted artifact exists.

r42 (648417) ran Replay through reference-fidelity failures for diagnosis;
these failures still make the run unsuccessful. Every measured source boundary
from frame 47 through 76 had native thumb AND non-thumb contact. The actual bowl
rose 46.9 mm from the local start. At core exit, object error was 59.0 mm /
0.907 rad, and the original handoff failed. Contact is not force-closure proof.
No state resets occurred within Replay. This isolates a carrying/pose-tracking
failure after acquisition, rather than an inability to make contact at all.

The diagnostic actual-state video has 47 source intervals at 30 fps:
`outputs/oracle_floating_a1_complete_replay_r42/replay/floating_xhand_actual.mp4`.
It is a failed local experiment, not a full 4.8 s successful rollout.

r43 (648426) tested an additional physical MPC proposal: capture the measured
hand–object transform on contact and derive Anchor wrist targets from the local
object reference. This is an engineering proposal, not a claim of paper-exact
MPPI reproduction. It changes neither object state nor RELAXED planning.
A mistakenly added immediate grasp-retention gate rejected acquisition at
frame 48; that gate has been removed. Original core fidelity and exit-contact
acceptance remain. Its trace also exposes an 18.6 mm / 0.141 rad target jump
at the switch, so target continuity remains a concrete candidate failure risk.
The remaining MPPI/PPO search did not produce an accepted A1.

r44 (648432) is a short correction attribution run with two MPPI candidates,
one iteration and PPO disabled only for this diagnostic. Default development
still uses Replay → MPPI → small PPO. It is submitted to interactive / H200,
20-minute allocation; final physical result is pending.

All changes are shared mechanisms: compatible free initial posture, residual
control semantics, collecting unsuccessful Replay evidence and measured grasp
proposals. No sample-name branch or pour/put-in-only controller rule is added.
The oracle schedule and raw local source data remain experiment inputs.
A1 is not yet physically accepted; A2/A3 and uninterrupted V0 remain incomplete.


## r44 result; r45 rotational velocity interface attribution

r44 completed in 52 s, with PPO deliberately omitted from this short probe.
Removing the unintended acquisition-retention gate lets the measured-grasp
proposal reach frame 51, versus nominal Replay's frame 49. It still fails the
unchanged core object fidelity. At frame 51, object error is 30.5 mm / 0.289 rad;
native opposition contact is absent. Improvement is not an accepted grasp.

The original floating wrench controller consumes only the linear half of the
six-dimensional velocity command. Angular damping always targeted zero,
although the local source and cuRobo controls provide a world angular velocity.
Measured nominal wrist rotation error grows 0.185 → 0.283 rad across frames
48–52 while commanded angular motion exceeds 2 rad/s. The r45 change tracks
`omega_target - omega_actual` with the existing rotational damping gain.
All gains, force/torque clamps, contact/object acceptance and timing are fixed.
This deliberately extends the older reproduction controller to honor the
current six-dimensional source-clock command contract.

r45 (648438) is Replay-only physical attribution, interactive H200 / 20 min.
The object-relative candidate's switching discontinuity remains a separate
observed risk; it is not used by this experiment. Result pending.


## r45–r47 outcome and consolidated review

r45 completed in 61.5 s. Nominal wrist rotation error at frame 52 decreased
from 0.283 to 0.185 rad, but object error and contact did not improve enough;
Replay still first failed at frame 49 and exit handoff failed at 76.
The 47-frame / 30-fps actual-state video was rendered and verified.

r46 completed its short prediction probe without PPO. Matching the target
switch removes the 18.6 mm / 0.141 rad jump; first switch increment becomes
0.906 mm / 0.00735 rad. The candidate still fails at frame 51, with object
error 34.9 mm / 0.280 rad (weighted error 1.061). This is not A1 success.

At the user's request, r47 / 648449 was cancelled while queued. A consolidated
review then fixed the active local synthesis/execution chain together; see
`docs/FLOATING_FLOW_AUDIT_20261005.md`. New changes include measured first-step
velocity matching, control-target continuity, retaining MPPI lookahead/PPO
initialization, keeping grasp drive targets across bridges, updating measured
collision geometry, permitting release inside release Anchors, fresh core-exit
export and a floating continuous executor. There is still no physically
accepted A1 or successful full floating V0 rollout.

Focused existing/regression checks: 21 passed in 0.57 s; Python syntax, shell
syntax and diff checks pass. These are interface checks, not physical evidence.
The next experiment evaluates the entire merged A1 synthesis chain once.


## r48: planning rejection before Anchor; preserve M1 during local research

Merged run r48 / 648475 failed in 71.4 s before executing the predecessor.
Uniform fitting reports velocity ratio 0.377 / acceleration ratio 6.771;
path-speed fallback rejects the initial tangent/velocity compatibility.
It did not execute Replay, MPC or PPO and provides no new interaction evidence.
The actual initial wrist pose/twist and all finger q/dq exactly match r45.

Offline retiming of the saved passed r45 path succeeds with acceleration ratio
0.50014. Overwriting that saved path's final target also succeeds, so this
attribution does not establish the cause of r48's different planner proposal.
Do not report a confirmed endpoint-overwrite cause. Default predecessor endpoint
processing is restored to the earlier behavior.

A generic `--predecessor-run` option now freezes a passed cuRobo development
bridge. Reuse requires matching source provenance, scene/contact/embodiment,
actual initial state within 1e-5, the exact source tick count, unchanged nominal
time fitting and current collision checks. The bridge is then physically
re-executed; no simulator state is copied or assigned. This is a RELAXED artifact,
not a complete robot demonstration. New sources still plan their own bridge.
The upcoming interaction experiment uses the passed r45 bridge as its baseline.

The new floating continuous executor and Hopper path are implemented and
syntax/contract checked, but no full floating V0 success is claimed.


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
