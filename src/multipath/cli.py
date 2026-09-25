"""Run V0 against a *real, externally supplied* Isaac/PhysX adapter."""
from __future__ import annotations
import argparse
import importlib
from pathlib import Path
from .schema import Anchor
from .orchestrator import run_single
from .planning.curobo import CuRoboPlanner


def load_factory(value: str):
    if ":" not in value:
        raise ValueError("use module:factory")
    module, name = value.split(":", 1)
    return getattr(importlib.import_module(module), name)


def main() -> int:
    p = argparse.ArgumentParser(description="One-path physics-verified Put-In")
    p.add_argument("--world", required=True, help="module:factory returning live Isaac/PhysX adapter")
    p.add_argument("--grasp-anchor", type=Path, required=True)
    p.add_argument("--place-anchor", type=Path, required=True)
    p.add_argument("--robot-yaml", required=True)
    p.add_argument("--world-yaml", required=True)
    p.add_argument("--attached-mesh", required=True)
    p.add_argument("--attached-pose-wxyz", nargs=7, type=float, required=True)
    p.add_argument("--output-dir", type=Path, required=True)
    args = p.parse_args()
    grasp, place = Anchor.load(args.grasp_anchor), Anchor.load(args.place_anchor)
    world = load_factory(args.world)()
    planner = CuRoboPlanner(args.robot_yaml, args.world_yaml,
                            held_object_mesh=args.attached_mesh,
                            held_object_pose=args.attached_pose_wxyz)
    result = run_single(world, planner, grasp, place, args.output_dir)
    print(f"success={result.success} stage={result.stage} trace={args.output_dir}")
    return 0 if result.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
