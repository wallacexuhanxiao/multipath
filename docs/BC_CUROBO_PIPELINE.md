# BC windows with cuRobo between them

The input is one egocentric video and its reconstruction. A VLM proposes
interaction windows; contact/object motion and task semantics check those
boundaries. For the current Put-In example, the existing selective-BC schedule
has BC at `[54,65)` and `[86,122)` (30 fps), with relaxed windows `[0,54)`,
`[65,86)`, and `[122,144)`. This unmodified VLM output is copied to
`configs/put_in_bc_curobo_schedule.json`. However, the prior PhysX contact
proxy first shows sustained grasp contact at frame 46. The original VLM BC
start at 54 would put frames 46-53 in a cuRobo window, so
`configs/put_in_bc_curobo_contact_checked.json` provisionally expands BC to
`[43,65)`. This boundary still needs video and mounted-robot confirmation.
`multipath.schedule.Schedule` maps
`RELAXED` to the intended cuRobo controller and validates complete coverage.

The VLM schedule only says **where** BC is needed. It contains no demonstrated
robot action and no verified entry or exit state. The execution pipeline is:

`scripts/prepare_egoengine_bc_windows.py` extracts the two local XHand/video
references using the contact-checked candidate to
`outputs/putin_bc_bootstrap_contact_checked/`. The XHand finger commands can
be retained when XHand is mounted; floating-root motion becomes arm-wrist
targets. Coupled-arm replay and physical contact must still be verified.

1. For each BC window, look for a demonstration from the same robot and scene.
   If absent, use the local video reference and EgoEngine method: split into
   chunks, retarget hand/object motion to that mounted robot, physically replay,
   run local MPC on failed chunks, then local RL only for remaining failures.
2. Replay the resulting BC candidate locally and record the actual robot,
   object, contact, and environment state at entry and exit. A single seed
   demonstration may be replayed for V0; a robust learned BC controller needs
   more demonstration coverage. Either way, the physical exit check is required.
3. For each relaxed window, use cuRobo from the actual preceding state to a
   reachable entry for the next BC window. Before grasp, plan without an
   attached object; while carrying, include the attached object's collision
   geometry and preservation constraints. Prefix and suffix windows need
   explicit initial/terminal conditions.
4. Execute BC/curobo/BC/curobo segments sequentially in the *same* PhysX
   instance. At each handoff, compare the measured state with the next BC
   entry set and physically correct or reject. A local segment pass is only
   diagnostic; count a path only after uninterrupted task success.
5. For later solutions, vary the cuRobo path in relaxed windows, keeping the
   BC interaction conditions. Reject near-duplicate object paths and all
   incomplete or physically failed executions.

The old `run_selective_bc_rl.py` uses RL for relaxed windows. The intended
replacement is cuRobo in **every** relaxed window; EgoEngine RL remains an
optional local tool to bootstrap a missing BC interaction. The current
`multipath.orchestrator.run_single` only covers two fixed BC anchors and their
middle transport. It is not yet a schedule-wide executor or BC trainer.

On Hopper, the pre-existing Put-In trajectory has 19 floating-hand DOFs per
XHand (root pose plus 12 finger DOFs). The initial Franka two-finger proxy was
an arm/planning smoke test, not the correct final hand embodiment. Retain the
XHand finger BC and use a mounted arm, with a fixed wrist-to-hand transform,
to follow its root pose. That needs reachability, self/world collision and
actual tracking checks. The Franka arm can reach all 58 tested BC wrist poses
under one provisional mount/scene transform; the selected 30 fps joint path
exceeds the Franka URDF velocity limits and needs trajectory repair. The original right XHand is
physically mounted with stable idle fixed-joint state. Finger actions map to
the mounted chain by name, but its controller still requires measured tuning;
the old gains failed on the combined articulation. The earlier two-finger
grasp probes do not test the mounted-XHand BC. See
`docs/HOPPER_FRANKA_PROBE.md` for measured outcomes and exact artifacts.
