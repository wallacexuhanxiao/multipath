# Previous Put-in reproduction versus current mounted V0

Verified against `egoengine复现&rl` and the local artifact:
`diversityverifier_phase1_scoped/outputs/putin_latest_onetomany_212ffca_r2/baseline/artifacts/refine_metrics.json`.
Same data: (put in, bowl, plate) / 20231024_253.

| Component | Previous successful reference baseline | Current V0 through r31 |
| --- | --- | --- |
| Wrist physics | Floating XHand root, external-wrench PD | Physical Franka + XHand; IK and joint command fitting |
| Displayed arm | Visualization-only in the cited realistic-arm Demo | Part of articulation dynamics |
| Solvers actually used | Replay, MPC at frames 40-60 and 120-140; no PPO selected | Replay/MPPI fail; 60-update small PPO repeatedly fails |
| MPC search | 64 candidates/environments, 3 iterations | 6 candidates, 2 iterations, serial |
| MPC action | Wrist xyz + rotation vector + 12 fingers | Wrist xyz + 12 fingers; rotation was omitted |
| Tracking gate | Independent .10 m / .90 rad bounds | Weighted scalar boundary with .05 m / .35 rad axial budgets |
| Contact in baseline metrics | Fingertip-to-mesh distance proxy, .015 m | Actual PhysX thumb AND other-finger force |
| PPO default batch | 256 environments; not needed by this baseline | Four serial episodes per update |
| Entry and reference | Physical sequential full reference rollout | r10 bridge-acquired local entry, padded local synthesis only |

The old baseline executed 144 frames and passed all committed chunks under
its own criteria. It does not demonstrate success under the new criteria.
For example its MPC chunk 40-60 had max tool translation .05221 m and rotation
.62241 rad; these exceed the current individual axial budgets. No new run is
needed to establish that the definitions are different.

The current core wrist prior also differs substantially from raw retarget:
r31 frame 40 has ~71.93 mm / 30.35 degree wrist error while arm command tracking
is accurate. The paper-style small residual operates around that fitted prior.
It is not established that current action limits and physical entry permit it
to repair the interaction. This is a hypothesis requiring a controlled test.

Correction: porting H=20, 2H lookahead and reward formulas did not amount to
reproducing the previous successful experiment. Mounted execution is an
intentional project change; smaller search budgets, omitted MPC orientation,
and different thresholds must be stated separately. Asking for a small network
did not inherently call for reducing rollout data to four episodes per update.

This turn restores the omitted world-frame MPC rotation residual (.2 rad scale)
with the original zero-residual bypass preserved. The focused selection test
also verifies nonzero rotational candidates reach IK. No reward, threshold,
physics, source-clock, or model-size change accompanies this correction.
Physical benefit remains untested; no new Hopper job was launched this turn.

Next useful comparison: reuse the known successful local interaction as a
DIAGNOSTIC baseline, then isolate mounted IK/fitting and actual execution.
Do not make a full robot reference a dependency of the final V0 runtime. Report
old and current tracking scores separately rather than silently loosening gates.
