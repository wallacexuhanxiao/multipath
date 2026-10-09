"""URDF finger kinematics for planning proposals; never assign simulator state."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def joint_model(path):
    root=ET.parse(path).getroot();joints=[]
    for node in root.findall('joint'):
        origin=node.find('origin');axis=node.find('axis')
        xyz=np.fromstring(origin.get('xyz','0 0 0') if origin is not None else '0 0 0',sep=' ')
        rpy=np.fromstring(origin.get('rpy','0 0 0') if origin is not None else '0 0 0',sep=' ')
        transform=np.eye(4);transform[:3,3]=xyz;transform[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix()
        joints.append(dict(name=node.get('name'),kind=node.get('type'),parent=node.find('parent').get('link'),
            child=node.find('child').get('link'),origin=transform,
            axis=np.fromstring(axis.get('xyz','1 0 0') if axis is not None else '1 0 0',sep=' ')))
    children={j['child'] for j in joints}
    base=next(link.get('name') for link in root.findall('link') if link.get('name') not in children)
    return base,joints


def link_poses(model,values):
    base,joints=model;poses={base:np.eye(4)};pending=list(joints)
    while pending:
        ready=[j for j in pending if j['parent'] in poses]
        if not ready:raise ValueError('Disconnected hand URDF')
        for j in ready:
            motion=np.eye(4);angle=values.get(j['name'],0.)
            if j['kind'] in ('revolute','continuous'):motion[:3,:3]=Rotation.from_rotvec(j['axis']*angle).as_matrix()
            elif j['kind']=='prismatic':motion[:3,3]=j['axis']*angle
            poses[j['child']]=poses[j['parent']]@j['origin']@motion;pending.remove(j)
    return poses


def hand_sweep_meshes(scene,goal,meshes):
    from multipath.xhand import finger_joint_names
    from multipath.execution.mounted_scene import pose_matrix
    from pxr import UsdGeom
    start=scene.state();q0=np.asarray(start['hand_q']);goal=np.asarray(goal)
    count=max(2,int(np.ceil(np.max(np.abs(goal-q0))/np.deg2rad(10)))+1)
    model=joint_model(scene.args.xhand_urdf)
    names=finger_joint_names(getattr(scene,'side','right'))
    fk0=link_poses(model,dict(zip(names,q0)))
    inverse=np.linalg.inv(pose_matrix(start['wrist_xyz_wxyz']));cache=UsdGeom.XformCache();local=[]
    for prim,points,faces in meshes:
        link=str(prim.GetPath())[len(scene.root)+1:].split('/')[0]
        if link not in fk0:raise ValueError(f'Imported hand link missing in URDF: {link}')
        transform=np.linalg.inv(fk0[link])@inverse@np.asarray(cache.GetLocalToWorldTransform(prim)).T
        local.append((link,points@transform[:3,:3].T+transform[:3,3],faces))
    result=[]
    for t in np.linspace(0,1,count):
        fk=link_poses(model,dict(zip(names,q0+t*(goal-q0))))
        for link,points,faces in local:
            transform=fk[link]
            result.append((points@transform[:3,:3].T+transform[:3,3],faces))
    return result,count


def hand_sweep_spheres(scene,goal,meshes,cover):
    geometry,count=hand_sweep_meshes(scene,goal,meshes)
    return [sphere for vertices,faces in geometry for sphere in cover(vertices,faces=faces)],count
