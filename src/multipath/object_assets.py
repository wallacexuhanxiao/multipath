"""Prepare source objects without requiring a previously rendered sample."""
from pathlib import Path


def prepare_source_meshes(objects, cache):
    """Use the existing EgoEngine reproduction's conversion and mass assumptions."""
    import trimesh
    from egoengine.sim.scene import MASS_BY_NAME, MASS_DEFAULT

    assets = {}
    for role, obj in objects.items():
        folder = Path(cache) / obj.object_id
        folder.mkdir(parents=True, exist_ok=True)
        mesh_path = folder / (obj.object_id + '_m.obj')
        if not mesh_path.exists():
            mesh = trimesh.load(obj.mesh_path, force='mesh')
            mesh.apply_scale(.01)
            mesh.export(mesh_path)
        assets[role] = dict(id=obj.object_id, name=obj.name,
                            source_mesh_cm=obj.mesh_path, mesh_m=str(mesh_path),
                            mass_kg=MASS_BY_NAME.get(obj.name, MASS_DEFAULT),
                            mass_provenance='EgoEngine reproduction category assumption; TACO has no measured masses')
    return assets


def ensure_object_usds(cache, assets):
    """Run after SimulationApp starts, before any task physics is initialized."""
    missing = [(role, value) for role, value in assets.items()
               if not (Path(cache) / value['id'] / (value['id'] + '.usd')).exists()]
    if not missing:
        return
    import os
    import numpy as np
    import trimesh
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema

    for role, value in missing:
        folder = Path(cache) / value['id']
        print('SOURCE_OBJECT_CONVERSION_BEGIN', role, value['id'], flush=True)
        mesh = trimesh.load(value['mesh_m'], force='mesh')
        destination = folder / (value['id'] + '.usd')
        temporary = folder / (value['id'] + f'.{os.getpid()}.usd')
        stage = Usd.Stage.CreateNew(str(temporary))
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        root = UsdGeom.Xform.Define(stage, '/Object').GetPrim()
        stage.SetDefaultPrim(root)
        UsdPhysics.RigidBodyAPI.Apply(root)
        UsdPhysics.MassAPI.Apply(root).CreateMassAttr(float(value['mass_kg']))
        geometry = UsdGeom.Mesh.Define(stage, '/Object/mesh')
        geometry.CreatePointsAttr(np.asarray(mesh.vertices, dtype=np.float32).tolist())
        geometry.CreateFaceVertexCountsAttr([3] * len(mesh.faces))
        geometry.CreateFaceVertexIndicesAttr(np.asarray(mesh.faces).ravel().tolist())
        geometry.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        geometry.CreateDoubleSidedAttr(True)
        # Match the existing cache's dynamic convex-decomposition model.
        # Author native USD directly; this backend uses Isaac Sim, not IsaacLab.
        UsdPhysics.CollisionAPI.Apply(geometry.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(geometry.GetPrim()).CreateApproximationAttr('convexDecomposition')
        PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(geometry.GetPrim())
        stage.GetRootLayer().Save()
        del stage
        os.replace(temporary, destination)
        print('SOURCE_OBJECT_CONVERSION_READY', role, value['id'], flush=True)
