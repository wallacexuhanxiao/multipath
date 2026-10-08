"""Local object-aware XHand initialization for elongated grasp regions.

MANO's scaled fingertip targets can sit centimetres outside a thin handle.
Project the local human grasp onto opposite mesh surfaces, then fit the robot
tips. This produces commands only, with no physical validity claim.
"""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation,Slerp
from scipy.ndimage import distance_transform_edt,map_coordinates
from pathlib import Path
from multipath.execution.mounted_scene import pose_matrix
from multipath.planning.hand_geometry import joint_model,link_poses
from multipath.xhand import RIGHT_XHAND_FINGER_JOINT_NAMES,RIGHT_XHAND_CLOSING_DIRECTION


def tool_surface_distance(mesh,cache=None):
    """Local fitting distance to the actual tool, including handle concavities.

    A whole-tool convex hull fills the empty space alongside a hammer handle.
    Use a mesh-sampled narrow-band field instead. This is a command proposal
    model; native PhysX contact remains the physical acceptance condition.
    """
    import hashlib,trimesh
    pitch=.001
    key=hashlib.sha256(mesh.vertices.tobytes()+mesh.faces.tobytes()).hexdigest()[:16]
    path=Path(cache)/(key+'.npz') if cache is not None else None
    if path is not None and path.exists():
        with np.load(path) as value:field=value['distance'];origin=value['origin']
    else:
        voxels=mesh.voxelized(pitch=pitch).fill();solid=np.pad(voxels.matrix,5)
        origin=voxels.transform[:3,3]-5*pitch
        field=(distance_transform_edt(~solid)-distance_transform_edt(solid))*pitch
        # Away from the surface the voxel field only supplies a broad phase.
        # Around contact, sample the original mesh's signed distance directly.
        indices=np.argwhere(np.abs(field)<.005)
        for begin in range(0,len(indices),4096):
            batch=indices[begin:begin+4096]
            field[tuple(batch.T)]=-trimesh.proximity.signed_distance(mesh,origin+batch*pitch)
        field=field.astype(np.float32)
        if path is not None:
            path.parent.mkdir(parents=True,exist_ok=True)
            np.savez_compressed(path,distance=field,origin=origin)
    def distance(points):
        return map_coordinates(field,((np.asarray(points)-origin)/pitch).T,order=1,mode='nearest')
    return distance


def surface_grasp_prior(anchor,frames,reference,human,objects,mesh,urdf,*,fps,table_z=None,physical_entry_tool=None,geometry_cache=None):
    if (anchor.entry_contact,anchor.exit_contact)!=('free','grasped'):return reference,None
    _,values,axes=np.linalg.svd(mesh.vertices-mesh.vertices.mean(0),full_matrices=False)
    if values[0]/values[1]<2:return reference,None
    index=anchor.core_end-1-anchor.synth_start
    source=np.asarray(reference[index]);tool=objects[index]
    human_local=(human[index]-tool[:3,3])@tool[:3,:3]
    long=axes[0]
    direction=human_local[4]-np.mean(human_local[[8,12]],axis=0)
    direction-=long*np.dot(direction,long)
    if np.linalg.norm(direction)<1e-6:return reference,None
    direction/=np.linalg.norm(direction);targets=[]
    for i,j in enumerate([4,8,12,16,20]):
        section=mesh.section(plane_normal=long,plane_origin=human_local[j])
        if section is None:return reference,None
        points=np.asarray(section.vertices);center=points.mean(0)
        radial=(1 if i==0 else -1)*direction
        point=points[np.argmax((points-center)@radial)]
        # A position-drive grasp needs a small contact preload. These are
        # command targets; PhysX, not this IK, determines actual penetration.
        targets.append(point-.0005*radial)
    targets=np.asarray(targets);model=joint_model(urdf)
    raw=np.linalg.inv(tool)@pose_matrix(source[:7]);q0=source[7:].copy()
    entry_tool=objects[anchor.core_start-anchor.synth_start]
    geometry_entry=(entry_tool if physical_entry_tool is None else np.asarray(physical_entry_tool))
    nodes={n.get('name'):n for n in ET.parse(urdf).getroot().findall('joint')}
    limits=np.asarray([[float(nodes[n].find('limit').get(k)) for k in ['lower','upper']]
        for n in RIGHT_XHAND_FINGER_JOINT_NAMES])
    tips=['thumb_rota_link2','index_rota_link2','mid_link2','ring_link2','pinky_link2']
    # The tiny tip links are tracking markers, not the robot's contact pads.
    # Fitting them alone placed whole distal phalanges inside the tool.
    import trimesh
    surfaces={};surface_radii={};contact_triangles={}
    def triangle_cover(triangles,radius=.002):
        centers=[];radii=[];pending=triangles
        while len(pending):
            center=pending.mean(1);size=np.linalg.norm(pending-center[:,None],axis=2).max(1)+1e-7
            done=size<=radius;centers.append(center[done]);radii.append(size[done])
            pending=pending[~done]
            if not len(pending):break
            edge=np.linalg.norm(np.roll(pending,-1,axis=1)-pending,axis=2).argmax(1);row=np.arange(len(pending))
            a=pending[row,edge];b=pending[row,(edge+1)%3];c=pending[row,(edge+2)%3];mid=(a+b)/2
            pending=np.concatenate((np.stack((a,mid,c),axis=1),np.stack((mid,b,c),axis=1)))
        return np.concatenate(centers),np.concatenate(radii)
    for link in ET.parse(urdf).getroot().findall('link'):
        visual=link.find('visual');geometry=link.find('visual/geometry/mesh')
        if geometry is None:continue
        path=Path(urdf).parent/geometry.get('filename')
        hull=trimesh.load(path,force='mesh').convex_hull
        points=np.array(hull.vertices,copy=True);triangles=np.array(hull.triangles,copy=True)
        origin=visual.find('origin')
        if origin is not None:
            points=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).apply(points)
            points+=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
            triangles=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).apply(triangles.reshape(-1,3)).reshape(triangles.shape)
            triangles+=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
        # Sparse vertices missed crossing phalange faces and tool ends.
        # Every triangle is covered here, including its sphere radius.
        surfaces[link.get('name')],surface_radii[link.get('name')]=triangle_cover(triangles)
        contact_triangles[link.get('name')]=triangles
    # Grasp targets are local; collision geometry retains the entire tool.
    # Its actual concavities must remain available for handle opposition.
    tool_distance=tool_surface_distance(mesh,geometry_cache)
    radii=np.concatenate(list(surface_radii.values()))
    contact_links={'right_hand_'+t for t in tips}
    lengths=[len(p) for p in surfaces.values()];cuts=np.cumsum(lengths)[:-1]
    contact_mask=np.concatenate([np.full(len(p),name in contact_links) for name,p in surfaces.items()])
    def link_penalties(values):
        # Mesh subdivision is a geometric resolution, not an objective
        # weight. 150k surface samples must not outweigh five contact goals.
        return np.asarray([np.max(v,initial=0.) for v in np.split(values,cuts)])
    def pose(x):
        result=raw.copy();result[:3,3]+=x[:3]
        result[:3,:3]=Rotation.from_rotvec(x[3:6]).as_matrix()@raw[:3,:3]
        return result
    def residual(x):
        fk=link_poses(model,dict(zip(RIGHT_XHAND_FINGER_JOINT_NAMES,x[6:])))
        world={name:points@(pose(x)@fk[name])[:3,:3].T+(pose(x)@fk[name])[:3,3]
               for name,points in surfaces.items()}
        p=[]
        for name,target in zip(tips,targets):
            name='right_hand_'+name;matrix=pose(x)@fk[name];triangles=contact_triangles[name]
            local=(target-matrix[:3,3])@matrix[:3,:3]
            nearest=trimesh.triangles.closest_point(triangles,np.repeat(local[None],len(triangles),axis=0))
            point=nearest[np.argmin(np.linalg.norm(nearest-local,axis=1))]
            p.append(point@matrix[:3,:3].T+matrix[:3,3])
        p=np.asarray(p)
        points=np.concatenate(list(world.values()))
        outside=tool_distance(points)-np.where(contact_mask,0.,radii)
        # Contact pads may drive slightly into the surface to create preload;
        # the rest of the hand remains outside the tool. Neither is a state
        # assignment or a replacement for actual physical contact validation.
        margin=np.concatenate([np.full(len(points),-.0005 if name in contact_links else .0005)
                               for name,points in world.items()])
        # Contact targets intentionally preload a distal surface. Its
        # circumscribed sampling sphere is not physical finger penetration.
        # Non-contact links retain conservative spheres; native PhysX still
        # determines whether any resulting grasp proposal actually works.
        collision=link_penalties(np.maximum(margin-outside,0))/.002
        table=(link_penalties(np.maximum(table_z+.001+radii-(points@geometry_entry[:3,:3].T+geometry_entry[:3,3])[:,2],0))/.002
               if table_z is not None else np.empty(0))
        return np.r_[((p-targets)*np.array([2,1,1,.5,.5])[:,None]/.004).ravel(),collision,table,
                     x[:3]/.08,x[3:6]/.9,(x[6:]-q0)/.8]
    initial=np.r_[np.zeros(6),np.clip(q0,limits[:,0]+1e-8,limits[:,1]-1e-8)]
    initial_fk=link_poses(model,dict(zip(RIGHT_XHAND_FINGER_JOINT_NAMES,q0)))
    reach=max(np.linalg.norm(p@initial_fk[n][:3,:3].T+initial_fk[n][:3,3],axis=1).max()
              for n,p in surfaces.items())
    fit=least_squares(residual,initial,bounds=(np.r_[np.full(3,-reach),np.full(3,-np.pi),limits[:,0]],
        np.r_[np.full(3,reach),np.full(3,np.pi),limits[:,1]]),max_nfev=120)
    delta=np.linalg.inv(raw)@pose(fit.x)
    # The human's wrist frame changes while the fingers wrap. Continuing to
    # track that scaled frame undoes the surface fit before the lift. Use the
    # local object-relative grasp proposal; physical acquisition still decides
    # the transform used for the subsequent interaction and bridges.
    # Before acquisition the tool is physically resting. Following its future
    # source motion here pushed it away before thumb opposition existed. The
    # actual acquired-grasp controller takes over source tracking afterwards.
    # A uniform all-joint opening required >1 rad of closure on short Anchors.
    # Solve the nearest free robot posture instead, with a hard joint travel
    # envelope for this unchanged CORE budget. The free bridge prepares it.
    fps=float(fps)
    closure_s=(anchor.core_end-anchor.core_start)/(2*fps)
    velocity=np.asarray([float(nodes[n].find('limit').get('velocity')) for n in RIGHT_XHAND_FINGER_JOINT_NAMES])
    travel=.98*velocity*closure_s/1.5
    closed=pose(fit.x);closed_q=fit.x[6:]
    def pre_pose(x):
        matrix=closed.copy();matrix[:3,3]+=x[:3]
        matrix[:3,:3]=Rotation.from_rotvec(x[3:6]).as_matrix()@closed[:3,:3]
        return matrix
    def pre_points(x):
        fk=link_poses(model,dict(zip(RIGHT_XHAND_FINGER_JOINT_NAMES,x[6:])))
        return np.concatenate([p@(pre_pose(x)@fk[n])[:3,:3].T+(pre_pose(x)@fk[n])[:3,3]
                               for n,p in surfaces.items()])
    def pre_residual(x):
        points=pre_points(x)
        distance=tool_distance(points)-radii
        table=(np.maximum(table_z+.001+radii-(points@geometry_entry[:3,:3].T+geometry_entry[:3,3])[:,2],0)/.0005
               if table_z is not None else np.empty(0))
        return np.r_[x[:3]/.02,x[3:6]/.15,(x[6:]-closed_q)/travel,
                     np.maximum(.002-distance,0)/.0005,table]
    low=np.maximum(limits[:,0],closed_q-travel);high=np.minimum(limits[:,1],closed_q+travel)
    initial=np.r_[np.zeros(6),np.clip(closed_q-.1*np.asarray(RIGHT_XHAND_CLOSING_DIRECTION),low+1e-9,high-1e-9)]
    prep=least_squares(pre_residual,initial,bounds=(np.r_[np.full(3,-.06),np.full(3,-.35),low],
        np.r_[np.full(3,.06),np.full(3,.35),high]),max_nfev=100)
    points=pre_points(prep.x)
    pregrasp_clearance=float(np.min(tool_distance(points)-radii))
    phase=np.clip(2*(frames-anchor.core_start)/(anchor.core_end-anchor.core_start),0,1)
    phase=phase**2*(3-2*phase)
    a=entry_tool@pre_pose(prep.x);b=entry_tool@closed
    result=np.asarray(reference).copy();result[:,:3]=a[:3,3]+phase[:,None]*(b[:3,3]-a[:3,3])
    result[:,3:7]=Slerp([0,1],Rotation.from_matrix([a[:3,:3],b[:3,:3]]))(phase).as_quat()[:,[3,0,1,2]]
    opened=prep.x[6:]
    result[:,7:]=opened+phase[:,None]*(closed_q-opened)
    result[:,7:]=np.clip(result[:,7:],limits[:,0],limits[:,1])
    return result,dict(method='local_mesh_opposition_surface_IK',physical_acceptance=False,
        geometry_aspect_ratio=float(values[0]/values[1]),source_frame=int(frames[index]),
        contact_target_object_xyz=targets.tolist(),contact_rms_m=float(np.sqrt(np.mean(
            (residual(fit.x)[:15].reshape(5,3)/np.array([2,1,1,.5,.5])[:,None])**2))*.004),
        collision_model='nonconvex tool SDF; distal contact surfaces, conservative other-link spheres; proposal only',
        geometry_objective='worst clearance per link, independent of surface sample count',
        wrist_translation_search_bound_m=float(reach),
        tool_distance_grid_pitch_m=.001,hand_surface_cover_spheres=len(radii),
        pre_acquisition_tool_motion='stationary_entry',
        free_pregrasp_surface_clearance_m=pregrasp_clearance,
        planned_closure_s=closure_s,pregrasp_max_joint_travel_rad=float(np.max(np.abs(closed_q-opened))),
        wrist_local_correction=delta.tolist())
