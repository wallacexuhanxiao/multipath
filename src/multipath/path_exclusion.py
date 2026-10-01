"""Turn accepted measured paths into cumulative virtual planning obstacles."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def transport_points(trace, start, end):
    points = np.asarray([r['wrist_xyz_wxyz'][:3] for r in trace
                         if (r['mode'] == 'CUROBO' and start <= r['source_frame'] <= end)
                         or r['source_frame'] in (start,end)], dtype=float)
    if len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('No finite measured cuRobo path for requested window')
    return points


def resample(points, count=101):
    points = np.asarray(points, dtype=float)
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    keep = np.r_[True, np.diff(distance) > 1e-9]
    if distance[-1] < 1e-5:
        raise ValueError('Path has no measurable translation')
    query = np.linspace(0, distance[-1], count)
    return np.stack([np.interp(query, distance[keep], points[keep, k]) for k in range(3)], axis=1)


def interval_distance_profile(points, peak_ratio=.45, target_arc_ratio=.75,
                              max_half_size=.08, size_multiplier=1.):
    """Scale a tapered exclusion and visible-arc target to this measured path.

    The chord controls ordinary transfers. Half the traveled arc length keeps
    curved or near-looping intervals from collapsing to a tiny obstacle.
    Physical planning still decides whether the requested size is feasible.
    """
    path=np.asarray(points,dtype=float)
    if path.ndim!=2 or path.shape[1]!=3 or len(path)<2 or not np.isfinite(path).all():
        raise ValueError('Expected a finite Nx3 interval path')
    if not (0<peak_ratio<=1 and 0<target_arc_ratio<=2 and max_half_size>0 and 0<size_multiplier<=1):
        raise ValueError('Invalid distance-adaptive obstacle parameters')
    length=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum())
    chord=float(np.linalg.norm(path[-1]-path[0]))
    if length<1e-5:raise ValueError('Path has no measurable translation')
    scale=max(chord,.5*length)
    peak=min(max_half_size,peak_ratio*scale)*size_multiplier
    return {'path_length_m':length,'endpoint_distance_m':chord,
            'interval_distance_scale_m':scale,'requested_max_half_size_m':peak,
            'minimum_arc_distance_m':target_arc_ratio*scale,
            'peak_ratio':peak_ratio,'target_arc_ratio':target_arc_ratio,
            'size_multiplier':size_multiplier}


def obstacles_along_path(points, half_size=.025, spacing=None, boundary_spheres=None, clearance=.005):
    """Reusable geometry-only API: any world-space Nx3 path, no task/frame IDs.

    Sample the whole curve by arc length, then omit boxes intersecting the
    protected entry/exit geometry. If robot geometry is unavailable, protect
    endpoint balls of radius twice the obstacle half-size. No middle fraction
    or scene-specific obstacle coordinates are used.
    """
    points = np.asarray(points, dtype=float)
    spacing = half_size if spacing is None else spacing
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('Expected a finite Nx3 path with at least two points')
    if half_size <= 0 or spacing <= 0 or spacing > 2*half_size or clearance < 0:
        raise ValueError('Invalid obstacle size, spacing, or boundary clearance')
    length = np.linalg.norm(np.diff(points, axis=0), axis=1).sum()
    candidates = resample(points, max(2, int(np.ceil(length/spacing))+1))
    endpoints = np.c_[points[[0,-1]], np.full(2, 2*half_size)]
    spheres = endpoints if boundary_spheres is None else np.concatenate([endpoints, np.asarray(boundary_spheres).reshape(-1,4)])
    spheres = spheres[spheres[:,3]>0]
    if not np.isfinite(spheres).all():raise ValueError('Nonfinite boundary geometry')
    delta = np.maximum(np.abs(candidates[:,None,:]-spheres[None,:,:3])-half_size,0)
    distances = (np.linalg.norm(delta,axis=-1)-spheres[None,:,3]).min(axis=1)
    centers = candidates[distances>clearance]
    if not len(centers):raise ValueError('No room for path obstacles outside protected boundaries')
    boxes = {f'path_exclusion_{j}':{'dims':[2*half_size]*3,'pose':[*p.tolist(),1.,0.,0.,0.]}
             for j,p in enumerate(centers)}
    return boxes, {'path_length_m':float(length),'candidate_count':len(candidates),
                   'boundary_filtered_count':int(np.sum(distances<=clearance)),
                   'protected_sphere_count':len(spheres),'minimum_boundary_clearance_m':float(distances[distances>clearance].min())}


def tapered_obstacles_along_path(points, max_half_size=.025, boundary_spheres=None,
                                 clearance=.005, oriented=False):
    """Small -> large -> small envelope in normalized path arc length.

    The sine-squared profile is capped by measured endpoint geometry, then
    constrained to a single peak. No object names, frame IDs, or coordinates.
    Obstacles are virtual exclusions used only during their planning interval.
    """
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<2 or not np.isfinite(points).all():
        raise ValueError('Expected a finite Nx3 path with at least two points')
    if max_half_size<=0 or clearance<0:raise ValueError('Invalid obstacle size or clearance')
    length=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum())
    count=max(33,int(np.ceil(length/(max_half_size/2)))+1)
    centers=resample(points,count)
    tangent=np.gradient(centers,axis=0)
    tangent/=np.maximum(np.linalg.norm(tangent,axis=1,keepdims=True),1e-9)
    basis=np.eye(3)[np.argmin(np.abs(tangent),axis=1)]
    normal=np.cross(tangent,basis);normal/=np.linalg.norm(normal,axis=1,keepdims=True)
    rotation=np.stack([tangent,normal,np.cross(tangent,normal)],axis=-1)
    longitudinal_half=max(.003,.8*length/(count-1))
    u=np.linspace(0,1,count)
    requested=max_half_size*np.sin(np.pi*u)**2
    endpoints=np.c_[points[[0,-1]],np.full(2,min(max_half_size/2,length/10))]
    spheres=endpoints if boundary_spheres is None else np.concatenate([endpoints,np.asarray(boundary_spheres).reshape(-1,4)])
    spheres=spheres[spheres[:,3]>0]
    if not np.isfinite(spheres).all():raise ValueError('Nonfinite boundary geometry')
    def distances(sizes):
        if oriented:
            relative=spheres[None,:,:3]-centers[:,None,:]
            local=np.einsum('csi,cij->csj',relative,rotation)
            half=np.stack([np.full(count,longitudinal_half),sizes,sizes],axis=1)
            delta=np.maximum(np.abs(local)-half[:,None,:],0)
        else:
            delta=np.maximum(np.abs(centers[:,None,:]-spheres[None,:,:3])-sizes[:,None,None],0)
        return (np.linalg.norm(delta,axis=-1)-spheres[None,:,3]).min(axis=1)
    lo=np.zeros(count);hi=requested.copy()
    for _ in range(32):
        mid=(lo+hi)/2;safe=distances(mid)>clearance
        lo=np.where(safe,mid,lo);hi=np.where(safe,hi,mid)
    peak=count//2
    # Bound every earlier/later size by its neighbors towards the peak,
    # retaining the small-large-small shape even near boundary geometry.
    sizes=lo.copy()
    sizes[:peak+1]=np.minimum.accumulate(sizes[:peak+1][::-1])[::-1]
    sizes[peak:]=np.minimum.accumulate(sizes[peak:])
    keep=(sizes>=max_half_size*.1)&(distances(sizes)>clearance)
    if not keep.any():raise ValueError('No room for tapered path obstacles outside protected boundaries')
    quaternions=Rotation.from_matrix(rotation).as_quat()[:,[3,0,1,2]]
    boxes={f'path_exclusion_{j}':{
        'dims':([float(2*longitudinal_half),float(2*h),float(2*h)] if oriented else [float(2*h)]*3),
        'pose':[*p.tolist(),*(quaternions[i].tolist() if oriented else [1.,0.,0.,0.])]}
           for j,(i,p,h) in enumerate(zip(np.flatnonzero(keep),centers[keep],sizes[keep]))}
    return boxes,{'path_length_m':length,'candidate_count':count,'boundary_filtered_count':int((~keep).sum()),
                  'protected_sphere_count':len(spheres),'size_profile':'sin_squared_geometry_capped_unimodal',
                  'box_orientation':'path_tangent' if oriented else 'axis_aligned',
                  'longitudinal_half_size_m':longitudinal_half if oriented else None,
                  'arc_fractions':u[keep].tolist(),'half_sizes_m':sizes[keep].tolist(),
                  'requested_max_half_size_m':max_half_size,
                  'minimum_boundary_clearance_m':float(distances(sizes)[keep].min())}


def exclusion_segments(config):
    """Read new multi-window configurations and earlier single-window records."""
    return config.get('segments', [config])


def protect_live_wrist_boundaries(segment, start_xyz, end_xyz, radius=.008):
    """Rebuild this interval's tapered exclusions around live wrist anchors.

    Preserve every prior path in the same interval, while ensuring the current
    planning start and fixed end are outside the artificial exclusion tube.
    """
    spheres=np.c_[np.asarray([start_xyz,end_xyz]),np.full(2,radius)]
    boxes={};diagnostics=[]
    for index,source in enumerate(segment['sources']):
        prior=source['placement_diagnostics']
        maximum=prior.get('requested_max_half_size_m',prior.get('half_size_m'))
        if maximum is None:raise ValueError('Missing obstacle peak size')
        generated,stats=tapered_obstacles_along_path(
            source['wrist_path_xyz'],maximum,spheres,
            oriented=prior.get('box_orientation')=='path_tangent')
        boxes.update({f'path_exclusion_{index}_{j}':box for j,box in enumerate(generated.values())})
        diagnostics.append(stats)
    result={**segment,'cuboid':boxes,'live_boundary_spheres':spheres.tolist(),
            'live_boundary_diagnostics':diagnostics}
    # Endpoint protection must not silently erase the diversity constraint.
    for source in segment['sources']:
        if compare_path(np.asarray(source['wrist_path_xyz']),result)['actual_wrist_exclusion_pass']:
            raise ValueError('Protected boundaries removed exclusion of a prior path')
    return result


def exclusions_for_window(config, start, end):
    """Activate only this interval's accumulated virtual obstacles.

    Geometry from another interval never transfers, even if spatial paths
    overlap. Returning None means this interval has no virtual exclusions.
    """
    if config is None:return None
    matches=[segment for segment in exclusion_segments(config) if segment['window']==[start,end]]
    if len(matches)>1:raise ValueError('Duplicate exclusion interval')
    return matches[0] if matches else None


def recorded_boundary_spheres(directory, trace, start, end):
    """Recover protected robot/held-object geometry from actual mesh records."""
    geometry=np.load(directory/'visual_meshes.npz',allow_pickle=False)
    transforms=np.load(directory/'visual_actual.npz',allow_pickle=False)['transforms']
    names=json.loads((directory/'visual_mesh_paths.json').read_text())
    times=np.asarray([r['source_frame'] for r in trace])
    frames=[int(np.abs(times-frame).argmin()) for frame in (start,end)]
    arrays=[]
    for i,name in enumerate(names):
        if not name.startswith(('/World/Franka/','/xhand_right/','/World/tool/')):continue
        vertices=geometry[f'v{i}']
        for frame in frames:
            t=transforms[frame,i];points=vertices@t[:3,:3].T+t[:3,3]
            center=points.mean(0);_,_,axes=np.linalg.svd(points-center,full_matrices=False)
            coords=(points-center)@axes.T;lo=coords.min(0);hi=coords.max(0)
            counts=np.maximum(1,np.ceil((hi-lo)/.025).astype(int));width=(hi-lo)/counts
            grid=np.indices(tuple(counts)).reshape(3,-1).T
            centers=center+(lo+(grid+.5)*width)@axes
            radius=float(np.linalg.norm(width)*.5+.001)
            arrays.append(np.c_[centers,np.full(len(centers),radius)])
    if not arrays:raise ValueError('No recorded robot geometry for boundaries')
    return np.concatenate(arrays)


def _boundaries(directory, report, window_index):
    start=directory/f'planner_start_spheres_{window_index}.npz'
    if not start.exists():start=directory/f'source_start_spheres_{window_index}.npz'
    files = [start]
    end = directory/f'planner_end_spheres_{window_index}.npz'
    if end.exists():files.append(end)
    elif (directory/'visual_actual.npz').exists():
        window=report['windows'][window_index]
        trace=json.loads((directory/'actual_trace.json').read_text())
        measured=recorded_boundary_spheres(directory,trace,window['start'],window['end'])
        arrays=[np.load(p,allow_pickle=False)['spheres'].reshape(-1,4) for p in files if p.exists()]
        return np.concatenate([measured,*arrays]), ['visual_actual.npz','visual_meshes.npz',*[p.name for p in files if p.exists()]]
    else:
        # Older accepted records saved the last feasible precontact endpoint.
        for j,candidate in enumerate(report['windows'][window_index].get('precontact_candidates', [])):
            if candidate[1]=='None':files.append(directory/f'candidate_spheres_{window_index}_{j}.npz')
    arrays = [np.load(p,allow_pickle=False)['spheres'].reshape(-1,4) for p in files if p.exists()]
    return (np.concatenate(arrays) if arrays else None), [p.name for p in files if p.exists()]


def build_exclusions(run_dirs, start=None, end=None, half_size=.025, adaptive_size=False,
                     exclusion_scope="wrist", window_policy="all", size_mode="fixed",
                     peak_ratio=.45, target_arc_ratio=.75, max_half_size=.08,
                     size_multiplier=1.):
    if not run_dirs or (start is None)!=(end is None):
        raise ValueError('Invalid exclusion geometry, window, or empty accepted path list')
    if size_mode not in ('fixed','distance') or (size_mode=='fixed' and not 0<half_size<=.1):
        raise ValueError('Invalid exclusion size mode or fixed size')
    if size_mode=='distance':
        adaptive_size=True
    if window_policy not in ('all','longest-budget'):raise ValueError('Unknown window policy')
    if exclusion_scope!='wrist':raise ValueError('Virtual path obstacles constrain only the wrist')
    result = {'version': 2, 'exclusion_scope':exclusion_scope, 'window_policy':window_policy,
              'size_mode':size_mode,'kind': 'virtual_planner_obstacles',
              'half_size_m': half_size if size_mode=='fixed' else None,
              'placement': 'tapered_arc_length_with_boundary_geometry_filter' if adaptive_size else 'arc_length_with_boundary_geometry_filter',
              'sources': [], 'segments': []}
    selected = None
    for index, directory in enumerate(run_dirs):
        directory = Path(directory)
        report = json.loads((directory / 'pipeline_result.json').read_text())
        if not report.get('full_pipeline_success'):
            raise ValueError(f'{directory}: source is not a full successful rollout')
        if not report.get('reference_semantics_validated'):
            raise ValueError(f'{directory}: source semantics and timing have not been validated against the video')
        if report.get('path_exclusions') and not report.get('diverse_solution_success'):
            raise ValueError(f'{directory}: source did not pass its path exclusion checks')
        scene = {key: report[key] for key in ('source_sha256', 'mount_xyzw', 'scene_translation_xyz')}
        if index and scene != result['scene']:
            raise ValueError('Accepted paths come from different scenes or references')
        result['scene'] = scene
        trace_path = directory / 'actual_trace.json'
        trace = json.loads(trace_path.read_text())
        metadata={'run':directory.name,'pbs_job_id':report.get('pbs_job_id'),
                  'trace_sha256':hashlib.sha256(trace_path.read_bytes()).hexdigest()}
        result['sources'].append(metadata)
        windows=report['windows']
        if selected is None:
            if start is not None:selected=[(start,end)]
            else:
                interior=[w for i,w in enumerate(windows) if w['controller']=='CUROBO' and 0<i<len(windows)-1
                          and windows[i-1]['controller']=='BC' and windows[i+1]['controller']=='BC']
                chosen=interior or [w for w in windows if w['controller']=='CUROBO']
                if window_policy=='longest-budget':chosen=[max(chosen,key=lambda w:w['end']-w['start'])]
                selected=[(w['start'],w['end']) for w in chosen]
            if not selected:raise ValueError('No cuRobo intervals found in accepted rollout')
            result['segments']=[{'window':list(w),'sources':[],'cuboid':{}} for w in selected]
        for segment in result['segments']:
            a,b=segment['window']
            match=[i for i,w in enumerate(windows) if (w['start'],w['end'])==(a,b) and w['controller']=='CUROBO']
            if len(match)!=1:raise ValueError('Accepted paths have incompatible motion intervals')
            path=transport_points(trace,a,b)
            if size_mode=='distance':
                if 'distance_profile' not in segment:
                    segment['distance_profile']=interval_distance_profile(
                        path,peak_ratio,target_arc_ratio,max_half_size,size_multiplier)
                    segment['minimum_arc_distance_m']=segment['distance_profile']['minimum_arc_distance_m']
                peak=segment['distance_profile']['requested_max_half_size_m']
            else:peak=half_size
            spheres,files=None,[]
            if adaptive_size:
                boxes,stats=tapered_obstacles_along_path(path,peak,boundary_spheres=spheres,
                                                         oriented=size_mode=='distance')
            else:
                boxes,stats=obstacles_along_path(path,peak,boundary_spheres=spheres)
                stats['half_size_m']=peak
            endpoint=min(trace,key=lambda r:abs(r['source_frame']-b))
            if 'boundary_wrist_xyz_wxyz' not in segment:
                segment['boundary_wrist_xyz_wxyz']=endpoint['wrist_xyz_wxyz']
                if 'arm_q' in endpoint:segment['boundary_arm_q']=endpoint['arm_q']
            segment['sources'].append({**metadata,'wrist_path_xyz':resample(path).tolist(),
                                       'boundary_geometry_files':files,'placement_diagnostics':stats})
            segment['cuboid'].update({f'path_exclusion_{index}_{j}':box for j,box in enumerate(boxes.values())})
    return result


def box_clearance(points, boxes):
    """Signed point distance to the union of oriented exclusion boxes."""
    points = np.asarray(points)
    distances = []
    for box in boxes.values():
        quaternion=np.asarray(box['pose'][3:])
        rotation=Rotation.from_quat(quaternion[[1,2,3,0]])
        local=rotation.inv().apply(points-np.asarray(box['pose'][:3]))
        delta = np.abs(local) - np.asarray(box['dims']) / 2
        distances.append(np.linalg.norm(np.maximum(delta, 0), axis=-1) + np.minimum(np.max(delta, axis=-1), 0))
    if not distances:
        raise ValueError('No path exclusion boxes')
    return np.min(distances, axis=0)


def compare_path(points, exclusions):
    # Dense arc samples prevent a fast crossing between recorded frames from
    # appearing collision-free. Require more clearance than sampling spacing.
    length = np.linalg.norm(np.diff(points, axis=0), axis=1).sum()
    sampled = resample(points, max(101, int(np.ceil(length / .001)) + 1))
    clearance = float(box_clearance(sampled, exclusions['cuboid']).min())
    path = resample(points)
    comparisons = []
    for source in exclusions['sources']:
        old = np.asarray(source['wrist_path_xyz'])
        distances = np.linalg.norm(path[:, None, :] - old[None, :, :], axis=-1)
        nearest=distances.min(axis=1)
        comparisons.append({'source_run': source['run'],
                            'mean_nearest_path_distance_m': float(nearest.mean()),
                            'max_nearest_path_distance_m': float(nearest.max()),
                            'middle_median_path_distance_m': float(np.median(nearest[20:81])),
                            'symmetric_hausdorff_m': float(max(distances.min(axis=1).max(), distances.min(axis=0).max()))})
    target=float(exclusions.get('minimum_arc_distance_m',0.))
    arc_pass=all(c['max_nearest_path_distance_m']>=target and
                 c['middle_median_path_distance_m']>=.5*target for c in comparisons)
    chord=float(np.linalg.norm(path[-1]-path[0]))
    path_length=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum())
    progress=(path-path[0])@(path[-1]-path[0])/max(chord*chord,1e-10)
    backward_progress=float(np.maximum(-np.diff(progress),0).sum())
    endpoint_overshoot=float(max(0.,-progress.min(),progress.max()-1.))
    # A large distance caused by a loop is not the broad, simple arc sought by
    # distance mode. Keep older fixed-size exclusions free of this new gate.
    smooth_arc_pass=(target<=0 or (path_length<=3.25*max(chord,1e-5)
                                     and backward_progress<=.25
                                     and endpoint_overshoot<=.20))
    return {'actual_wrist_min_exclusion_clearance_m': clearance,
            'actual_wrist_exclusion_pass': clearance > .002,
            'minimum_arc_distance_m':target,'actual_arc_pass':arc_pass,
            'path_length_m':path_length,'endpoint_distance_m':chord,
            'backward_progress_fraction':backward_progress,
            'endpoint_overshoot_fraction':endpoint_overshoot,
            'smooth_arc_pass':smooth_arc_pass,
            'comparisons': comparisons}
