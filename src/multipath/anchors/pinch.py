"""Geometry-conditioned finger drive proposals, never simulator state writes."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from multipath.planning.hand_geometry import joint_model,link_poses
from multipath.xhand import finger_joint_names


def contact_linearization(urdf,distance,wrist_in_object,hand_q,opposing_finger,*,side='right'):
    """Linearize two real finger skins at a local grasp command.

    Positive flexion can move a fitted contact surface away from an object.
    The SDF derivative supplies a local proposal direction; it does not
    establish force closure or replace a fresh physical rollout.
    """
    import trimesh
    urdf=Path(urdf);fingers=('thumb',opposing_finger);surfaces={}
    for link in ET.parse(urdf).getroot().findall('link'):
        name=link.get('name');visual=link.find('visual');mesh=link.find('visual/geometry/mesh')
        if mesh is None or name.endswith('_tip') or not any(f in name for f in fingers):continue
        hull=trimesh.load(urdf.parent/mesh.get('filename'),force='mesh').convex_hull
        points=np.vstack((hull.vertices,hull.triangles.mean(1)))
        origin=visual.find('origin')
        if origin is not None:
            points=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).apply(points)
            points+=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
        surfaces[name]=points
    model=joint_model(urdf);wrist_in_object=np.asarray(wrist_in_object);hand_q=np.asarray(hand_q)
    def gaps(q):
        fk=link_poses(model,dict(zip(finger_joint_names(side),q)));values=[]
        for finger in fingers:
            points=[]
            for name,skin in surfaces.items():
                if finger not in name:continue
                matrix=wrist_in_object@fk[name]
                points.append(skin@matrix[:3,:3].T+matrix[:3,3])
            values.append(float(np.min(distance(np.concatenate(points)))))
        return np.asarray(values)
    step=.001;axes=np.eye(len(hand_q))*step
    gradient=np.stack([(gaps(hand_q+axis)-gaps(hand_q-axis))/(2*step) for axis in axes],axis=1)
    if gradient.shape!=(2,12) or not np.isfinite(gradient).all():
        raise ValueError('Invalid local finger surface derivative')
    return dict(contact_fingers=list(fingers),surface_gap_m=gaps(hand_q).tolist(),
                gap_gradient_m_per_rad=gradient.tolist(),physical_acceptance=False,
                role='local finger-drive initialization; native physics required')
