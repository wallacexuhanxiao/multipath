"""Import the existing right XHand and physically mount it to a Franka arm."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xhand-urdf", type=Path, required=True)
    parser.add_argument("--bc-reference", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    app = SimulationApp({"headless": True})
    result = {"status": "failed", "physical_xhand_mount_verified": False}
    try:
        import omni
        from isaacsim.asset.importer.urdf import _urdf
        from isaacsim.core.api import World
        from isaacsim.core.api.robots import Robot
        from isaacsim.core.prims import SingleXFormPrim
        from isaacsim.core.utils.nucleus import get_assets_root_path
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.utils.types import ArticulationAction
        from pxr import Gf, Sdf, UsdPhysics, PhysxSchema
        from scipy.spatial.transform import Rotation

        world = World(stage_units_in_meters=1.0, physics_dt=1 / 120)
        world.scene.add_default_ground_plane()
        assets = get_assets_root_path()
        if assets is None:
            raise RuntimeError("Franka USD asset unavailable")
        add_reference_to_stage(
            assets + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
            "/World/Franka",
        )
        franka = world.scene.add(Robot(prim_path="/World/Franka", name="franka"))

        importer = _urdf.acquire_urdf_interface()
        cfg = _urdf.ImportConfig()
        cfg.set_fix_base(False)
        cfg.set_merge_fixed_joints(False)
        cfg.set_self_collision(False)
        cfg.set_convex_decomp(False)
        imported = importer.parse_urdf(str(args.xhand_urdf.parent), args.xhand_urdf.name, cfg)
        imported_path = importer.import_robot(
            str(args.xhand_urdf.parent), args.xhand_urdf.name, imported, cfg,
            "",
        )
        stage = omni.usd.get_context().get_stage()
        result["xhand_stage_paths"] = [str(p.GetPath()) for p in stage.Traverse()
                                       if str(p.GetPath()).startswith(str(imported_path))][:100]
        xhand_links = [str(p.GetPath()) for p in stage.Traverse()
                       if str(p.GetPath()).startswith(str(imported_path) + "/")
                       and p.GetName() == "right_hand_link"]
        if len(xhand_links) != 1:
            raise RuntimeError(f"XHand base link unresolved: {xhand_links}")
        hand_mount = SingleXFormPrim("/World/Franka/panda_hand", name="franka_hand_mount")
        xhand_root = SingleXFormPrim(str(imported_path), name="xhand_asset_root")
        hand_p, hand_q = hand_mount.get_world_pose()
        xhand_root.set_world_pose(position=hand_p, orientation=hand_q)
        removed_articulation_roots = []
        for prim in stage.Traverse():
            if str(prim.GetPath()).startswith(str(imported_path)) and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                removed_articulation_roots.append(str(prim.GetPath()))
        joint_path = "/World/Franka/xhand_mount_joint"
        joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
        joint.CreateBody0Rel().SetTargets([Sdf.Path("/World/Franka/panda_hand")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(xhand_links[0])])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        # Match EgoEngine's force drives; the importer defaults can use
        # acceleration drives, for which the same numerical gains behave differently.
        drive_info = {}
        for prim in stage.Traverse():
            if str(prim.GetPath()).startswith(str(imported_path)) and prim.IsA(UsdPhysics.RevoluteJoint):
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                drive_info[prim.GetName()] = {"old_type": drive.GetTypeAttr().Get(),
                                              "old_stiffness": drive.GetStiffnessAttr().Get()}
                drive.CreateTypeAttr("force")
            if "panda_leftfinger" in str(prim.GetPath()) or "panda_rightfinger" in str(prim.GetPath()):
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateEnabledSelfCollisionsAttr(False)
        result["imported_drive_settings"] = drive_info
        world.reset()
        xhand_mount = SingleXFormPrim(xhand_links[0], name="xhand_root_mount")

        def relative_pose():
            p0, q0 = hand_mount.get_world_pose()
            p1, q1 = xhand_mount.get_world_pose()
            q0 = np.asarray(q0, dtype=float)
            q1 = np.asarray(q1, dtype=float)
            r0 = Rotation.from_quat(q0[[1, 2, 3, 0]])
            r1 = Rotation.from_quat(q1[[1, 2, 3, 0]])
            return np.r_[r0.inv().apply(np.asarray(p1) - np.asarray(p0)),
                         (r0.inv() * r1).as_quat()]

        initial = relative_pose()
        positions = []
        for _ in range(120):
            world.step(render=False)
            positions.append(relative_pose())
        positions = np.asarray(positions)
        drift = np.linalg.norm(positions[:, :3] - initial[:3], axis=1)
        result.update({
            "status": "mounted_idle_probe_complete",
            "imported_root": str(imported_path),
            "xhand_base_link": xhand_links[0],
            "franka_dof_names": list(franka.dof_names),
            "franka_dof_count": int(franka.num_dof),
            "removed_xhand_articulation_roots": removed_articulation_roots,
            "mount_initial_relative_pose_xyz_xyzw": initial.tolist(),
            "max_mount_position_drift_m": float(drift.max()),
            "fixed_joint_path": str(joint.GetPrim().GetPath()),
            "physical_xhand_mount_verified": bool(drift.max() < 0.002),
        })
        np.savez_compressed(args.output_dir / "mount_idle_trace.npz", relative_pose=positions)
        if args.bc_reference:
            from multipath.xhand import RIGHT_XHAND_FINGER_JOINT_NAMES
            with np.load(args.bc_reference, allow_pickle=False) as data:
                finger_commands = np.asarray(data["qpos_right"][:, 7:], dtype=float)
                source_finger_actual = np.asarray(data["qpos_right_actual"][:, 7:], dtype=float)
                source_frames = np.asarray(data["source_frame_indices"], dtype=int)
            dof_names = list(franka.dof_names)
            indices = np.array([dof_names.index(n) for n in RIGHT_XHAND_FINGER_JOINT_NAMES])
            art_view = franka._articulation_view
            art_view.set_gains(kps=np.full((1, len(indices)), 5.0),
                               kds=np.full((1, len(indices)), 0.5),
                               joint_indices=indices)
            art_view.set_max_efforts(np.full((1, len(indices)), 3.0),
                                     joint_indices=indices)
            franka.apply_action(ArticulationAction(
                joint_positions=finger_commands[0], joint_indices=indices))
            for _ in range(120):
                world.step(render=False)
            initial_finger_error = np.abs(
                np.asarray(franka.get_joint_positions(), dtype=float)[indices] - finger_commands[0])
            finger_actual = []
            mount_relative = []
            for command in finger_commands:
                franka.apply_action(ArticulationAction(
                    joint_positions=command, joint_indices=indices))
                for _ in range(4):
                    world.step(render=False)
                finger_actual.append(np.asarray(franka.get_joint_positions(), dtype=float)[indices])
                mount_relative.append(relative_pose())
            finger_actual = np.asarray(finger_actual)
            mount_relative = np.asarray(mount_relative)
            error = np.abs(finger_actual - finger_commands)
            source_error = np.abs(finger_actual - source_finger_actual)
            result.update({
                "status": "mounted_xhand_finger_bc_probe_complete",
                "bc_source_frames": source_frames.tolist(),
                "xhand_finger_dof_indices": indices.tolist(),
                "pre_bc_first_frame_settle_steps": 120,
                "pre_bc_first_frame_max_abs_error_rad": float(initial_finger_error.max()),
                "finger_command_max_abs_error_rad": float(error.max()),
                "finger_command_mean_abs_error_rad": float(error.mean()),
                "finger_vs_source_actual_max_abs_error_rad": float(source_error.max()),
                "finger_vs_source_actual_mean_abs_error_rad": float(source_error.mean()),
                "xhand_drive_kp_kd_max_force": [5.0, 0.5, 3.0],
                "max_mount_position_drift_during_bc_m": float(np.max(np.linalg.norm(
                    mount_relative[:, :3] - initial[:3], axis=1))),
                "arm_wrist_bc_tracking_tested": False,
                "object_contact_tested": False,
            })
            np.savez_compressed(args.output_dir / "mounted_finger_bc_trace.npz",
                                frames=source_frames, command=finger_commands,
                                actual=finger_actual, source_actual=source_finger_actual,
                                mount_relative_pose=mount_relative)
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        (args.output_dir / "mount_result.json").write_text(json.dumps(result, indent=2) + "\n")
        app.close()


if __name__ == "__main__":
    main()
