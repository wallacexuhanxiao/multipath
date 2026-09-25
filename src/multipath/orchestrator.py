"""Single-path, stateful, uninterrupted physics rollout; no intermediate reset."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Protocol
import json
import numpy as np

from .schema import Anchor, Boundary, Plan
from .transitions import Tolerance, check, static_anchor_compatibility


class PhysicsWorld(Protocol):
    """An adapter around the same physically simulated robot as cuRobo."""
    def reset_initial(self) -> Boundary: ...
    def observe(self) -> Boundary: ...
    def execute_joint_commands(self, commands: np.ndarray, *, label: str, dt: float | None = None) -> None: ...
    def correct_to(self, target: Boundary) -> bool: ...
    def check_grasp(self) -> bool: ...
    def check_placement(self) -> bool: ...
    def no_hard_physics_failure(self) -> bool: ...
    def save_actual_trace(self, folder: Path) -> None: ...


class Planner(Protocol):
    def plan(self, start: Boundary, goal: Boundary) -> Plan: ...


@dataclass(frozen=True)
class RunResult:
    success: bool
    stage: str
    detail: dict


def run_single(
    world: PhysicsWorld, planner: Planner, grasp: Anchor, place: Anchor,
    output_dir: str | Path, tol: Tolerance = Tolerance(),
) -> RunResult:
    """A *single continuous execution* from reset to placement.

    Standalone anchor tests run separately before this method. `correct_to`
    may issue only physical commands in the existing world and must not reset
    bodies, roots or contacts. This method invokes `reset_initial` only once.
    """
    folder = Path(output_dir)
    folder.mkdir(parents=True, exist_ok=True)
    stage = "preflight"
    report: dict = {"grasp_source": grasp.source_commit, "place_source": place.source_commit}
    try:
        if grasp.name != "grasp" or place.name != "put_in":
            raise ValueError("expected named grasp and put_in anchors")
        static_anchor_compatibility(grasp.exit, place.entry)
        if grasp.entry.scene_id != place.entry.scene_id:
            raise ValueError("initial / place scene mismatch")
        if (grasp.frame_range[1] > place.frame_range[0] or
                grasp.source_commit != place.source_commit):
            raise ValueError("overlapping anchors or different source provenance")
        stage = "reset_initial"
        start = world.reset_initial()  # only reset in full run
        gate = check(start, grasp.entry, tol)
        report["initial_gate"] = gate
        if not gate["ok"]:
            raise RuntimeError("initial measured state mismatches verified grasp entry")
        stage = "grasp"
        world.execute_joint_commands(grasp.commands, label="grasp")
        observed = world.observe()
        gate = check(observed, grasp.exit, tol)
        report["grasp_exit_gate"] = gate
        if not gate["ok"] or not world.check_grasp() or not world.no_hard_physics_failure():
            raise RuntimeError("grasp anchor did not physically reach verified exit")
        stage = "plan"
        path = planner.plan(observed, place.entry)
        if path.goal_arm_joint_names != observed.arm_joint_names:
            raise ValueError("planner changed joint order")
        report["plan_steps"] = len(path.arm_q)
        stage = "transport"
        hand_hold = np.broadcast_to(observed.hand_q, (len(path.arm_q), len(observed.hand_q)))
        commands = np.concatenate((path.arm_q, hand_hold), axis=1)
        world.execute_joint_commands(commands, label="relaxed_transport", dt=path.dt)
        if not world.check_grasp() or not world.no_hard_physics_failure():
            raise RuntimeError("lost grasp or physical safety during transport")
        stage = "transition"
        observed = world.observe()
        gate = check(observed, place.entry, tol)
        report["transition_gate_before"] = gate
        if not gate["ok"]:
            stage = "physical_correction"
            if not world.correct_to(place.entry):
                raise RuntimeError("real corrective motion unavailable/failed")
            if not world.check_grasp() or not world.no_hard_physics_failure():
                raise RuntimeError("correction invalidated grasp / safety")
            observed = world.observe()
            gate = check(observed, place.entry, tol)
        report["transition_gate_after"] = gate
        if not gate["ok"]:
            raise RuntimeError("actual robot never reached put-in canonical boundary")
        stage = "put_in"
        world.execute_joint_commands(place.commands, label="put_in")
        observed = world.observe()
        report["put_in_exit_gate"] = check(observed, place.exit, tol)
        # Final placement need not match human exact exit q, but it must be
        # physically safe and meet the functional task predicate.
        if not world.check_placement() or not world.no_hard_physics_failure():
            raise RuntimeError("physical Put-In task success not established")
        stage = "complete"
        result = RunResult(True, stage, report)
    except (RuntimeError, ValueError) as exc:
        report["failure"] = str(exc)
        result = RunResult(False, stage, report)
    finally:
        # Do not silently drop failure traces; reveal an unavailable trace.
        try:
            world.save_actual_trace(folder)
        except Exception as exc:
            report["trace_error"] = str(exc)
            if "result" in locals() and result.success:
                result = RunResult(False, "trace", report)
        (folder / "result.json").write_text(json.dumps(
            {"success": result.success if "result" in locals() else False,
             "stage": result.stage if "result" in locals() else stage,
             **report}, indent=2, allow_nan=False) + "\n")
    return result
