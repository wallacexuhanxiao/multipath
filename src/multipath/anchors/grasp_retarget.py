"""Local object-aware XHand initialization from source grasp geometry.

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
from multipath.xhand import finger_joint_names,RIGHT_XHAND_CLOSING_DIRECTION


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
        coordinates=((np.asarray(points)-origin)/pitch).T
        clipped=np.clip(coordinates,0,(np.asarray(field.shape)-1)[:,None])
        # A narrow field must not label a finger centimetres outside its
        # domain as merely 5 mm from the tool. Extend the proposal distance
        # monotonically outside the box; actual near-contact values remain
        # sampled from the original mesh field.
        return map_coordinates(field,clipped,order=1,mode='nearest')+np.linalg.norm(coordinates-clipped,axis=0)*pitch
    def normal(points):
        points=np.asarray(points)
        gradient=np.stack([(distance(points+axis*pitch)-distance(points-axis*pitch))/(2*pitch)
                           for axis in np.eye(3)],axis=1)
        length=np.linalg.norm(gradient,axis=1)
        missing=(length<1e-8)|np.any((points<origin)|(points>origin+(np.asarray(field.shape)-1)*pitch),axis=1)
        if np.any(missing):
            _,_,face=trimesh.proximity.closest_point(mesh,points[missing])
            gradient[missing]=mesh.face_normals[face];length[missing]=1.
        return gradient/length[:,None]
    distance.normal=normal
    return distance


def surface_grasp_prior(anchor,frames,reference,human,objects,mesh,urdf,*,fps,table_z=None,physical_entry_tool=None,geometry_cache=None,initial_grasp=None,side='right'):
    if (anchor.entry_contact,anchor.exit_contact)!=('free','grasped'):return reference,None
    names=finger_joint_names(side)
    import trimesh
    _,values,axes=np.linalg.svd(mesh.vertices-mesh.vertices.mean(0),full_matrices=False)
    index=anchor.core_end-1-anchor.synth_start
    source=np.asarray(reference[index]);tool=objects[index]
    human_local=(human[index]-tool[:3,3])@tool[:3,:3]
    targets=[]
    if values[0]/values[1]>=2:
        long=axes[0]
        direction=human_local[4]-np.mean(human_local[[8,12]],axis=0)
        direction-=long*np.dot(direction,long)
        if np.linalg.norm(direction)<1e-6:return reference,None
        direction/=np.linalg.norm(direction)
        for i,j in enumerate([4,8,12,16,20]):
            section=mesh.section(plane_normal=long,plane_origin=human_local[j])
            if section is None:return reference,None
            points=np.asarray(section.vertices);center=points.mean(0)
            radial=(1 if i==0 else -1)*direction
            point=points[np.argmax((points-center)@radial)]
            targets.append(point-.0005*radial)
        target_geometry='source-local opposing handle cross sections'
    else:
        # Compact/hollow objects need local surface opposition (e.g. across
        # a wall), not opposite extremes of the whole object. Select surfaces
        # near the human contact region using mesh normals, without action IDs.
        points,_,triangles=trimesh.proximity.closest_point(mesh,human_local[[4,8,12,16,20]])
        normal=mesh.face_normals[triangles[0]]
        targets.append(points[0]-.0005*normal)
        candidates=np.flatnonzero(mesh.face_normals@normal<-.25)
        if not len(candidates):return reference,None
        for j in [8,12,16,20]:
            nearest=trimesh.triangles.closest_point(mesh.triangles[candidates],
                np.repeat(human_local[j][None],len(candidates),axis=0))
            selected=np.argmin(np.linalg.norm(nearest-human_local[j],axis=1))
            targets.append(nearest[selected]-.0005*mesh.face_normals[candidates[selected]])
        target_geometry='source-local opposing nonconvex mesh surfaces'
    targets=np.asarray(targets);model=joint_model(urdf)
    raw=np.linalg.inv(tool)@pose_matrix(source[:7]);q0=source[7:].copy()
    entry_tool=objects[anchor.core_start-anchor.synth_start]
    geometry_entry=(entry_tool if physical_entry_tool is None else np.asarray(physical_entry_tool))
    nodes={n.get('name'):n for n in ET.parse(urdf).getroot().findall('joint')}
    limits=np.asarray([[float(nodes[n].find('limit').get(k)) for k in ['lower','upper']]
        for n in names])
    fingers=['thumb','index','mid','ring','pinky']
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
    contact_links={name for name in surfaces if any(finger in name for finger in fingers) and not name.endswith('_tip')}
    lengths=[len(p) for p in surfaces.values()];cuts=np.cumsum(lengths)[:-1]
    contact_mask=np.concatenate([np.full(len(p),name in contact_links) for name,p in surfaces.items()])
    finger_masks=[np.concatenate([np.full(len(points),name in contact_links and finger in name)
                  for name,points in surfaces.items()]) for finger in fingers]
    def finger_clearances(outside):
        # Physical opposition accepts any real phalange surface. The point
        # nearest a human tip is only a soft source target; requiring that
        # particular point to touch gave a different objective from PhysX.
        return np.asarray([np.min(outside[mask]) for mask in finger_masks])
    def finger_witnesses(points,outside):
        return np.asarray([points[np.flatnonzero(mask)[np.argmin(outside[mask])]] for mask in finger_masks])
    def link_penalties(values):
        # Mesh subdivision is a geometric resolution, not an objective
        # weight. 150k surface samples must not outweigh five contact goals.
        return np.asarray([np.max(v,initial=0.) for v in np.split(values,cuts)])
    def pose(x):
        result=raw.copy();result[:3,3]+=x[:3]
        result[:3,:3]=Rotation.from_rotvec(x[3:6]).as_matrix()@raw[:3,:3]
        return result
    def table_penalties(wrist,fk):
        if table_z is None:return np.empty(0)
        heights=[]
        for name,triangles in contact_triangles.items():
            matrix=geometry_entry@wrist@fk[name]
            vertices=triangles.reshape(-1,3)
            heights.append(np.min(vertices@matrix[2,:3]+matrix[2,3]))
        # For a plane, triangle extrema are exactly at vertices. Covering
        # sphere radii created fictitious table overlap during local closure.
        return np.maximum(table_z+.001-np.asarray(heights),0)
    def contact_points(x,fk):
        p=[]
        for finger,target in zip(fingers,targets):
            candidates=[]
            # Native grasp accepts real proximal or distal phalange contact.
            # The old proposal prohibited proximal contact while requiring
            # distal-only opposition, contradicting its physical objective.
            for name in sorted(contact_links):
                if finger not in name:continue
                matrix=pose(x)@fk[name];triangles=contact_triangles[name]
                local=(target-matrix[:3,3])@matrix[:3,:3]
                nearest=trimesh.triangles.closest_point(triangles,np.repeat(local[None],len(triangles),axis=0))
                point=nearest[np.argmin(np.linalg.norm(nearest-local,axis=1))]
                candidates.append(point@matrix[:3,:3].T+matrix[:3,3])
            p.append(min(candidates,key=lambda point:np.linalg.norm(point-target)))
        return np.asarray(p)
    def residual(x,opposition_finger=None):
        fk=link_poses(model,dict(zip(names,x[6:])))
        world={name:points@(pose(x)@fk[name])[:3,:3].T+(pose(x)@fk[name])[:3,3]
               for name,points in surfaces.items()}
        p=contact_points(x,fk)
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
        collision=link_penalties(np.maximum(margin-outside,0))/(.002 if opposition_finger is None else .0005)
        table=table_penalties(pose(x),fk)/.002
        # A small global fingertip RMS can still leave every opposing finger
        # millimetres outside a thin tool. Match the native grasp objective:
        # thumb AND at least one opposing finger must reach a contact surface.
        # This only proposes drives; fresh native contact/retention is required.
        opposition=np.empty(0)
        if opposition_finger is not None:
            contact_distance=finger_clearances(outside)
            normals=tool_distance.normal(finger_witnesses(points,outside)[[0,opposition_finger]])
            # Merely touching with two fingers can press both onto the upper
            # handle bevel and never support a lift. Fit opposite tool normals
            # as a geometry proposal; native physical retention is unchanged.
            opposition=np.r_[(contact_distance[[0,opposition_finger]]+.0005)/.0005,
                              (normals[0]+normals[1])/.2]
        return np.r_[((p-targets)*np.array([2,1,1,.5,.5])[:,None]/.004).ravel(),opposition,collision,table,
                     x[:3]/.08,x[3:6]/.9,(x[6:]-q0)/.8]
    initial=np.r_[np.zeros(6),np.clip(q0,limits[:,0]+1e-8,limits[:,1]-1e-8)]
    initial_fk=link_poses(model,dict(zip(names,q0)))
    reach=max(np.linalg.norm(p@initial_fk[n][:3,:3].T+initial_fk[n][:3,3],axis=1).max()
              for n,p in surfaces.items())
    bounds=(np.r_[np.full(3,-reach),np.full(3,-np.pi),limits[:,0]],
        np.r_[np.full(3,reach),np.full(3,np.pi),limits[:,1]])
    fit=least_squares(residual,initial,bounds=bounds,max_nfev=120)
    # First obtain a source-conditioned, collision-aware branch; then close
    # its nearest opposing surface gap. A changing min over fingers jumped
    # between unrelated contacts and produced an observed blocked free entry.
    fk=link_poses(model,dict(zip(names,fit.x[6:])))
    points=np.concatenate([points@(pose(fit.x)@fk[name])[:3,:3].T+(pose(fit.x)@fk[name])[:3,3]
                           for name,points in surfaces.items()])
    outside=tool_distance(points)-np.where(contact_mask,0.,radii)
    distance=finger_clearances(outside)
    normals=tool_distance.normal(finger_witnesses(points,outside))
    opposition_finger=1+int(np.argmin(np.abs(distance[1:]+.0005)/.004+
                                     np.linalg.norm(normals[1:]+normals[0],axis=1)/.2))
    fit=least_squares(lambda x:residual(x,opposition_finger),fit.x,bounds=bounds,max_nfev=120)
    seed_origin='source_retarget'
    if initial_grasp is not None:
        # A previous local proposal is a numerical seed, never a physical
        # entry state. Keep its better branch when the geometry model changes.
        previous=np.asarray(initial_grasp)
        wrist=np.linalg.inv(entry_tool)@pose_matrix(previous[:7])
        prior_x=np.r_[wrist[:3,3]-raw[:3,3],Rotation.from_matrix(wrist[:3,:3]@raw[:3,:3].T).as_rotvec(),previous[7:]]
        prior_x=np.clip(prior_x,bounds[0]+1e-8,bounds[1]-1e-8)
        fk=link_poses(model,dict(zip(names,prior_x[6:])))
        points=np.concatenate([points@(pose(prior_x)@fk[name])[:3,:3].T+(pose(prior_x)@fk[name])[:3,3]
                               for name,points in surfaces.items()])
        outside=tool_distance(points)-np.where(contact_mask,0.,radii)
        distance=finger_clearances(outside);normals=tool_distance.normal(finger_witnesses(points,outside))
        prior_finger=1+int(np.argmin(np.abs(distance[1:]+.0005)/.004+
                                    np.linalg.norm(normals[1:]+normals[0],axis=1)/.2))
        prior_score=np.sum(residual(prior_x,prior_finger)**2)
        if prior_score<np.sum(residual(fit.x,opposition_finger)**2):
            candidate=least_squares(lambda x:residual(x,prior_finger),prior_x,bounds=bounds,max_nfev=120)
            if np.sum(residual(candidate.x,prior_finger)**2)<=prior_score:
                fit=candidate
            else:fit.x=prior_x
            opposition_finger=prior_finger;seed_origin='previous_local_grasp_proposal'
    fit_diagnostics=residual(fit.x,opposition_finger)
    fk=link_poses(model,dict(zip(names,fit.x[6:])))
    points=np.concatenate([points@(pose(fit.x)@fk[name])[:3,:3].T+(pose(fit.x)@fk[name])[:3,3]
                           for name,points in surfaces.items()])
    outside=tool_distance(points)-np.where(contact_mask,0.,radii)
    normals=tool_distance.normal(finger_witnesses(points,outside)[[0,opposition_finger]])
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
    velocity=np.asarray([float(nodes[n].find('limit').get('velocity')) for n in names])
    travel=.98*velocity*closure_s/1.5
    closed=pose(fit.x);closed_q=fit.x[6:]
    def pre_pose(x):
        matrix=closed.copy();matrix[:3,3]+=x[:3]
        matrix[:3,:3]=Rotation.from_rotvec(x[3:6]).as_matrix()@closed[:3,:3]
        return matrix
    def pre_points(x):
        fk=link_poses(model,dict(zip(names,x[6:])))
        return np.concatenate([p@(pre_pose(x)@fk[n])[:3,:3].T+(pre_pose(x)@fk[n])[:3,3]
                               for n,p in surfaces.items()])
    def pre_residual(x):
        points=pre_points(x)
        distance=tool_distance(points)-radii
        fk=link_poses(model,dict(zip(names,x[6:])))
        table=table_penalties(pre_pose(x),fk)/.0005
        return np.r_[x[:3]/.02,x[3:6]/.15,(x[6:]-closed_q)/travel,
                     link_penalties(np.maximum(.002-distance,0))/.0005,table]
    low=np.maximum(limits[:,0],closed_q-travel);high=np.minimum(limits[:,1],closed_q+travel)
    initial=np.r_[np.zeros(6),np.clip(closed_q-.1*np.asarray(RIGHT_XHAND_CLOSING_DIRECTION),low+1e-9,high-1e-9)]
    prep=least_squares(pre_residual,initial,bounds=(np.r_[np.full(3,-reach),np.full(3,-np.pi),low],
        np.r_[np.full(3,reach),np.full(3,np.pi),high]),max_nfev=100)
    points=pre_points(prep.x)
    pregrasp_clearance=float(np.min(tool_distance(points)-radii))
    recovery=False
    if pregrasp_clearance<0:
        # Opposite sides of a thin wall can cancel local translation gradients.
        # Enter a free basin geometrically, then optimize toward the grasp;
        # this only initializes commands, never lifts or places the object.
        up=geometry_entry[:3,:3].T@np.array([0.,0.,1.])
        for fraction in [.05,.1,.25,.5,1.]:
            seed=prep.x.copy();seed[:3]=np.clip(seed[:3]+fraction*reach*up,-reach,reach)
            clearance=float(np.min(tool_distance(pre_points(seed))-radii))
            fk=link_poses(model,dict(zip(names,seed[6:])))
            if clearance<.002 or np.any(table_penalties(pre_pose(seed),fk)>0):continue
            candidate=least_squares(pre_residual,seed,bounds=(np.r_[np.full(3,-reach),np.full(3,-np.pi),low],
                np.r_[np.full(3,reach),np.full(3,np.pi),high]),max_nfev=100)
            refined=float(np.min(tool_distance(pre_points(candidate.x))-radii))
            fk=link_poses(model,dict(zip(names,candidate.x[6:])))
            if refined>=0 and not np.any(table_penalties(pre_pose(candidate.x),fk)>1e-6):
                prep=candidate;pregrasp_clearance=refined
            else:
                prep.x=seed;pregrasp_clearance=clearance
            recovery=True;break
    phase=np.clip(2*(frames-anchor.core_start)/(anchor.core_end-anchor.core_start),0,1)
    phase=phase**2*(3-2*phase)
    a=entry_tool@pre_pose(prep.x);b=entry_tool@closed
    result=np.asarray(reference).copy();result[:,:3]=a[:3,3]+phase[:,None]*(b[:3,3]-a[:3,3])
    result[:,3:7]=Slerp([0,1],Rotation.from_matrix([a[:3,:3],b[:3,:3]]))(phase).as_quat()[:,[3,0,1,2]]
    opened=prep.x[6:]
    result[:,7:]=opened+phase[:,None]*(closed_q-opened)
    result[:,7:]=np.clip(result[:,7:],limits[:,0],limits[:,1])
    from multipath.anchors.pinch import contact_linearization
    pinch=contact_linearization(urdf,tool_distance,closed,closed_q,fingers[opposition_finger],side=side)
    return result,dict(method='local_mesh_opposition_surface_IK',physical_acceptance=False,
        pinch_proposal=pinch,
        geometry_aspect_ratio=float(values[0]/values[1]),source_frame=int(frames[index]),
        contact_target_object_xyz=targets.tolist(),contact_rms_m=float(np.sqrt(np.mean(
            (fit_diagnostics[:15].reshape(5,3)/np.array([2,1,1,.5,.5])[:,None])**2))*.004),
        contact_target_geometry=target_geometry,
        nominal_opposition_surface_clearance_m=(fit_diagnostics[15:17]*.0005-.0005).tolist(),
        nominal_opposition_normal_dot=float(normals[0]@normals[1]),
        nominal_opposition_normals_object_frame=normals.tolist(),
        opposition_finger=fingers[opposition_finger],
        geometry_fit_seed=seed_origin,
        collision_model='nonconvex tool SDF; all real finger contact surfaces, conservative other-link spheres; proposal only',
        geometry_objective='worst clearance per link, independent of surface sample count',
        local_table_geometry='exact convex triangle vertices; no sphere-radius inflation',
        wrist_translation_search_bound_m=float(reach),
        pregrasp_translation_search_bound_m=float(reach),
        tool_distance_grid_pitch_m=.001,hand_surface_cover_spheres=len(radii),
        tool_distance_outside_grid='monotone boundary distance extension; original mesh face normals',
        pre_acquisition_tool_motion='stationary_entry',
        free_pregrasp_geometric_seed_recovery=recovery,
        free_pregrasp_surface_clearance_m=pregrasp_clearance,
        planned_closure_s=closure_s,pregrasp_max_joint_travel_rad=float(np.max(np.abs(closed_q-opened))),
        wrist_local_correction=delta.tolist())
