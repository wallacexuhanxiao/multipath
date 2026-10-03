# A2 / A3 readiness — 2026-10-03

CPU-only inspection of the existing Hopper inputs; no GPU job submitted by this preparation work.

## What is already ready

Root: `/scratch/e0985859/multipath_v0_fc2da7b`.

The existing `outputs/oracle_v0_putin_r10/proposals/manifest.json` matches the SHA-256 of `configs/oracle/put_in.json`. All three proposal archives exist and contain finite numerical data. These are local kinematic proposals, **not physically validated Anchor artifacts**.

| User label | ID | Core frames | Synthesis frames | Existing proposal | Required development entry |
|---|---|---|---|---|---|
| A1 | `anchor_0` | [35, 76) | [29, 82) | 53 frames | Frame 29: usable r10 physical predecessor |
| A2 | `anchor_1` | [82, 101) | [76, 107) | 31 frames | Frame 76: not yet available |
| A3 | `anchor_2` | [111, 128) | [105, 134) | 29 frames | Frame 105: not yet available |

Each archive has `source_frames`, `wrist_hand_q` (N × 19), `human_joints` (N × 21 × 3), and tool/target poses (N × 4 × 4). No retarget rerun is needed before attempting local physical synthesis.

`predecessor_snapshot` accepts A1's frame-29 state from `outputs/oracle_v0_putin_r10/development`. It rejects A2/A3 from that directory because the successful predecessor chain does not reach their synthesis starts. It also rejects `outputs/oracle_gate_parallel_r15/acquisition`: that run stopped at 0.0167 s and exported no snapshots. The old `outputs/source_clock_r11` is not a compatible snapshot source: its report lacks the required source provenance.

## Independent work that can run beside A1

Historical controls in `outputs/source_clock_r11/actual_trace.json` are available: 144 rows × 19 joints, source-frame coverage 0–143. They can be physically re-executed by the existing acquisition script to acquire frames 76 and 105. This is explicitly a development bootstrap, not the final Oracle runtime or evidence that A1/A2 succeeded. The current controller fixes and corrected acceleration policy will apply when this acquisition is rerun; its physical outcome remains unknown.

The commands below run **inside an allocated interactive GPU job**, using the same container environment and `run_sim` helper as `hpc/run_oracle_anchor_batch.pbs`. Do not execute simulation directly on the login node. Use a fresh `OUT`; do not overwrite prior evidence.

```bash
COMMON=(
  --proposals "$ROOT/outputs/oracle_v0_putin_r10/proposals"
  --schedule "$ROOT/configs/oracle/put_in.json"
  --robot-yaml "$ROOT/vendor/curobo-v077/src/curobo/content/configs/robot/franka.yml"
  --xhand-urdf "$ENGINE/assets/xhand/spider/xhand_right.urdf"
  --object-cache "$ENGINE/outputs/sim_usd"
)

# Independent of A1 local synthesis: measure actual states from real execution.
run_sim scripts/acquire_anchor_snapshots.py "${COMMON[@]}" \
  --command-trace "$ROOT/outputs/source_clock_r11/actual_trace.json" \
  --anchors anchor_1 anchor_2 --output-dir "$OUT/acquisition" \
  >"$OUT/acquisition.log" 2>&1

# Once valid physical inputs exist, each Anchor is an independent local job.
run_sim scripts/develop_mounted_anchor.py "${COMMON[@]}" \
  --anchor anchor_1 --predecessor-dir "$OUT/acquisition" \
  --artifact-dir "$OUT/anchors" --output-dir "$OUT/anchor_1" \
  >"$OUT/anchor_1.log" 2>&1
run_sim scripts/develop_mounted_anchor.py "${COMMON[@]}" \
  --anchor anchor_2 --predecessor-dir "$OUT/acquisition" \
  --artifact-dir "$OUT/anchors" --output-dir "$OUT/anchor_2" \
  >"$OUT/anchor_2.log" 2>&1
```

These two development commands can run concurrently if allocation memory allows; multiple Isaac processes are not free. Their actual entry provenance is checked before Isaac startup. If acquisition fails after frame 76 but before 105, its successfully recorded frame-76 chain can still support A2; inspect `pipeline_result.json` rather than treating all acquisition output as unusable. A physically acquired state can still be unsuitable for the intended grasp interaction: padded/core physical validation decides that.

Hopper `interactive` permits only one running job per user. Run acquisition and A1 within that same allocation when measured GPU/system-memory headroom permits, otherwise sequence them inside the allocation; do not submit additional interactive jobs expecting simultaneous execution. Both Python entry points use `multipath.execution.startup.start_simulation`, and the batch helper exports `PYTHONPATH=$ROOT/deps:$ROOT/src` and binds the corrected container startup environment.

## Dependency that cannot be parallelized away

Local A2/A3 artifacts do not prove final reachability. Final execution still requires actual A1 exit → cuRobo bridge → actual A2 entry → A2 → bridge → A3, with the original source clock and no state restore. A1's padded context overlaps subsequent source time; only its core is exported. Therefore, an endpoint of A1's padded run must not silently replace the final A1-core-to-A2 bridge.

Minimum next outputs: acquisition states at frames 76/105 with lineage; padded and exported-core results for each Anchor; then continuous prefix/full execution. No new analytical smoothing, RL, or extra acceptance gate is introduced here.
