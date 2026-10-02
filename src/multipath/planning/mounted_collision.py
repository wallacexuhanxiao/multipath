"""Existing measured XHand/carried-mesh sphere model, shared by oracle V0."""
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from pxr import UsdGeom

def mounted_collision_robot(robot_config, robot_yaml, world, flange, objects, meshes, carry):
    import copy
    from pxr import Usd
    cfg=copy.deepcopy(robot_config); kin=cfg['kinematics']
    spheres=yaml.safe_load((robot_yaml.parent/kin['collision_spheres']).read_text())['collision_spheres']
    for name in ('panda_leftfinger','panda_rightfinger'):
        kin['collision_link_names'].remove(name);spheres.pop(name,None)
    kin['lock_joints']={}
    for key in ('joint_names','retract_config','null_space_weight','cspace_distance_weight'):
        kin['cspace'][key]=kin['cspace'][key][:7]
    kin['self_collision_ignore']={k:[n for n in v if n in kin['collision_link_names']] for k,v in kin['self_collision_ignore'].items() if k in kin['collision_link_names']}
    kin['self_collision_buffer']={k:v for k,v in kin['self_collision_buffer'].items() if k in kin['collision_link_names']}
    fp,fq=flange.get_world_pose();fr=Rotation.from_quat(np.asarray(fq)[[1,2,3,0]])
    extras=[];cache=UsdGeom.XformCache()
    for prim in Usd.PrimRange.Stage(world.stage, Usd.TraverseInstanceProxies()):
        if not str(prim.GetPath()).startswith('/xhand_right') or not prim.IsA(UsdGeom.Mesh):continue
        if 'collision' in str(prim.GetPath()).lower():continue
        points=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
        trans=np.asarray(cache.GetLocalToWorldTransform(prim)).T
        points=points@trans[:3,:3].T+trans[:3,3]
        points=fr.inv().apply(points-np.asarray(fp))
        # Oriented boxes split along their longest axis, covered by spheres.
        center=points.mean(0);_,_,axes=np.linalg.svd(points-center,full_matrices=False)
        coords=(points-center)@axes.T;lo=coords.min(0);hi=coords.max(0)
        # Cover small oriented cells instead of a few oversized
        # spheres that protrude far beyond flat palm geometry.
        counts=np.maximum(1,np.ceil((hi-lo)/.025).astype(int))
        width=(hi-lo)/counts
        radius=float(np.linalg.norm(width)*.5+.001)
        for ijk in np.ndindex(*counts):
            c=lo+(np.asarray(ijk)+.5)*width
            extras.append({'center':(center+c@axes).tolist(),'radius':radius})
    if carry:
        op,oq=objects['tool'].get_world_pose();points=Rotation.from_quat(np.asarray(oq)[[1,2,3,0]]).apply(meshes['tool'].vertices)+op
        points=fr.inv().apply(points-np.asarray(fp))
        lo,hi=points.min(0),points.max(0);counts=np.maximum(1,np.ceil((hi-lo)/.025).astype(int))
        width=(hi-lo)/counts;radius=float(np.linalg.norm(width)*.5+.001)
        for ijk in np.ndindex(*counts):
            extras.append({'center':(lo+(np.asarray(ijk)+.5)*width).tolist(),'radius':radius})
    spheres['panda_hand']=extras
    kin['collision_spheres']=spheres
    return cfg,len(extras)
