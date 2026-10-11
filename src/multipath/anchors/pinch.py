"""Geometry-conditioned finger drive proposals, never simulator state writes."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from multipath.planning.hand_geometry import joint_model,link_poses
from multipath.xhand import finger_joint_names


def mesh_surface_distance(mesh):
    """Exact local skin queries; no whole-object millimetre field build."""
    import trimesh
    query=trimesh.proximity.ProximityQuery(mesh)
    def distance(points):
        points=np.asarray(points)
        # Native object meshes may contain tens of thousands of triangles.
        # Bound temporary nearest-triangle arrays for the initial skin scan.
        return np.concatenate([-query.signed_distance(points[i:i+64])
                               for i in range(0,len(points),64)])
    return distance


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
        # Mesh tessellation density is unrelated to MPC accuracy. Keep real
        # surface samples at the existing millimetre proposal resolution.
        _,indices=np.unique(np.round(points/.001).astype(np.int64),axis=0,return_index=True)
        points=points[indices]
        surfaces[name]=points
    model=joint_model(urdf);wrist_in_object=np.asarray(wrist_in_object);hand_q=np.asarray(hand_q)
    # Differentiate nearby actual skin witnesses, rather than re-querying
    # every hull vertex against a large tool mesh for each joint perturbation.
    # The sampled minimum at the measured pose is retained exactly. This model
    # only initializes candidates; fresh physics accepts the resulting grasp.
    fk=link_poses(model,dict(zip(finger_joint_names(side),hand_q)));selected={}
    for finger in fingers:
        points=[];locations=[]
        for name,skin in surfaces.items():
            if finger not in name:continue
            matrix=wrist_in_object@fk[name]
            points.append(skin@matrix[:3,:3].T+matrix[:3,3])
            locations.extend((name,j) for j in range(len(skin)))
        values=distance(np.concatenate(points))
        for index in np.argsort(values)[:8]:
            name,j=locations[index];selected.setdefault(name,[]).append(j)
    surfaces={name:surfaces[name][indices] for name,indices in selected.items()}
    def gaps(q,wrist=None):
        wrist=wrist_in_object if wrist is None else wrist
        fk=link_poses(model,dict(zip(finger_joint_names(side),q)));values=[]
        for finger in fingers:
            points=[]
            for name,skin in surfaces.items():
                if finger not in name:continue
                matrix=wrist@fk[name]
                points.append(skin@matrix[:3,:3].T+matrix[:3,3])
            values.append(float(np.min(distance(np.concatenate(points)))))
        return np.asarray(values)
    step=.001;axes=np.eye(len(hand_q))*step
    gradient=np.stack([(gaps(hand_q+axis)-gaps(hand_q-axis))/(2*step) for axis in axes],axis=1)
    if gradient.shape!=(2,12) or not np.isfinite(gradient).all():
        raise ValueError('Invalid local finger surface derivative')
    pose_gradient=[]
    for j in range(6):
        epsilon=.0005 if j<3 else .001
        perturbed=[]
        for sign in (1,-1):
            wrist=wrist_in_object.copy()
            if j<3:wrist[j,3]+=sign*epsilon
            else:
                angle=np.zeros(3);angle[j-3]=sign*epsilon
                wrist[:3,:3]=Rotation.from_rotvec(angle).as_matrix()@wrist[:3,:3]
            perturbed.append(gaps(hand_q,wrist))
        pose_gradient.append((perturbed[0]-perturbed[1])/(2*epsilon))
    return dict(contact_fingers=list(fingers),surface_gap_m=gaps(hand_q).tolist(),
                gap_gradient_m_per_rad=gradient.tolist(),
                wrist_gap_gradient_object_frame=np.asarray(pose_gradient).T.tolist(),physical_acceptance=False,
                local_skin_witnesses_per_finger=8,
                skin_sample_resolution_m=.001,
                role='local finger-drive initialization; native physics required')
