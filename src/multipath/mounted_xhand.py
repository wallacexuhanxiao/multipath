"""Isaac scene assembly. Import only after starting SimulationApp."""
from pathlib import Path
import numpy as np
from .xhand import RIGHT_XHAND_FINGER_JOINT_NAMES


def assemble(world, urdf_path, mount_xyzw):
    import omni
    from isaacsim.asset.importer.urdf import _urdf
    from isaacsim.core.api.robots import Robot
    from isaacsim.core.prims import SingleXFormPrim
    from isaacsim.core.utils.nucleus import get_assets_root_path
    from isaacsim.core.utils.stage import add_reference_to_stage
    from pxr import Gf, Sdf, Usd, UsdPhysics, PhysxSchema, UsdGeom
    from scipy.spatial.transform import Rotation
    stage = omni.usd.get_context().get_stage()
    add_reference_to_stage(get_assets_root_path() + '/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd', '/World/Franka')
    robot = world.scene.add(Robot(prim_path='/World/Franka', name='robot'))
    cfg = _urdf.ImportConfig()
    cfg.set_fix_base(False)
    cfg.set_merge_fixed_joints(False)
    cfg.set_self_collision(False)
    importer = _urdf.acquire_urdf_interface()
    urdf_path = Path(urdf_path).resolve()
    # The importer writes fixed-name *.tmp.usd files. Concurrent processes must
    # not create/delete those layers in the same working directory.
    import os,tempfile
    previous_directory=Path.cwd()
    import_directory=tempfile.mkdtemp(prefix='multipath-urdf-')
    try:
        os.chdir(import_directory)
        parsed = importer.parse_urdf(str(urdf_path.parent), urdf_path.name, cfg)
        root = str(importer.import_robot(str(urdf_path.parent), urdf_path.name, parsed, cfg, ''))
    finally:
        os.chdir(previous_directory)
    wrist = SingleXFormPrim(root + '/right_hand_link', name='wrist')
    flange = SingleXFormPrim('/World/Franka/panda_hand', name='flange')
    p, q = flange.get_world_pose()
    mounted_q = (Rotation.from_quat(np.asarray(q)[[1,2,3,0]]) * Rotation.from_quat(mount_xyzw)).as_quat()
    SingleXFormPrim(root).set_world_pose(position=p, orientation=mounted_q[[3,0,1,2]])
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.startswith(root):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            if prim.IsA(UsdPhysics.RevoluteJoint):
                drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
                drive.CreateTypeAttr('force')
        if prim.GetName() in ('panda_hand', 'panda_leftfinger', 'panda_rightfinger'):
            UsdGeom.Imageable(prim).MakeInvisible()
            # Deinstance the replaced gripper so collider overrides reach
            # its mesh descendants rather than stopping at instance roots.
            for child in Usd.PrimRange(prim):
                if child.IsInstance():child.SetInstanceable(False)
            for child in Usd.PrimRange(prim):
                if child.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(child).GetCollisionEnabledAttr().Set(False)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateEnabledSelfCollisionsAttr(False)
    joint = UsdPhysics.FixedJoint.Define(stage, '/World/Franka/xhand_mount')
    joint.CreateBody0Rel().SetTargets([Sdf.Path('/World/Franka/panda_hand')])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(root + '/right_hand_link')])
    joint.CreateLocalPos0Attr(Gf.Vec3f(0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0))
    joint.CreateLocalRot0Attr(Gf.Quatf(float(mount_xyzw[3]), *map(float, mount_xyzw[:3])))
    joint.CreateLocalRot1Attr(Gf.Quatf(1,0,0,0))
    return robot, wrist, flange


def configure_drives(robot):
    names = list(robot.dof_names)
    fingers = np.array([names.index(n) for n in RIGHT_XHAND_FINGER_JOINT_NAMES])
    arm = np.array([names.index(f'panda_joint{i}') for i in range(1,8)])
    view = robot._articulation_view
    view.set_gains(kps=np.full((1,12),5.), kds=np.full((1,12),.5), joint_indices=fingers)
    view.set_max_efforts(np.full((1,12),3.), joint_indices=fingers)
    return arm, fingers


class ActualMeshRecorder:
    """Save geometry and measured world transforms for offline rendering."""
    def __init__(self, stage, out):
        from pxr import Usd, UsdGeom
        self.stage, self.out = stage, Path(out)
        self.prims, self.geometry, self.meta = [], {}, []
        for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            mesh = UsdGeom.Mesh(prim)
            if UsdGeom.Imageable(prim).ComputeVisibility() == 'invisible':
                continue
            path = str(prim.GetPath())
            # Imported robot collision geometry must not be rendered twice.
            if 'collision' in path.lower() or 'defaultGroundPlane' in path:
                continue
            points = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float32)
            counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
            indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
            faces, offset = [], 0
            for n in counts:
                poly = indices[offset:offset+n]
                faces.extend([[poly[0],poly[k],poly[k+1]] for k in range(1,n-1)])
                offset += n
            if not len(faces):
                continue
            i = len(self.prims)
            self.prims.append(prim)
            self.geometry[f'v{i}'] = points
            self.geometry[f'f{i}'] = np.asarray(faces,dtype=np.int32)
            self.meta.append(path)
        np.savez_compressed(self.out/'visual_meshes.npz', **self.geometry)
        self.transforms = []

    def capture(self):
        from pxr import UsdGeom
        cache = UsdGeom.XformCache()
        self.transforms.append(np.stack([np.asarray(cache.GetLocalToWorldTransform(p)).T for p in self.prims]))

    def save(self):
        import json
        np.savez_compressed(self.out/'visual_actual.npz', transforms=np.asarray(self.transforms,dtype=np.float32))
        (self.out/'visual_mesh_paths.json').write_text(json.dumps(self.meta))
