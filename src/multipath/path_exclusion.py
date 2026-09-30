"""Turn accepted measured paths into cumulative virtual planning obstacles."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import numpy as np


def transport_points(trace, start, end):
    points = np.asarray([r['wrist_xyz_wxyz'][:3] for r in trace
                         if r['mode'] == 'CUROBO' and start <= r['source_frame'] <= end], dtype=float)
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


def exclusion_segments(config):
    """Read new multi-window configurations and earlier single-window records."""
    return config.get('segments', [config])


def _boundaries(directory, report, window_index):
    files = [directory/f'planner_start_spheres_{window_index}.npz']
    end = directory/f'planner_end_spheres_{window_index}.npz'
    if end.exists():files.append(end)
    else:
        # Older accepted records saved the last feasible precontact endpoint.
        for j,candidate in enumerate(report['windows'][window_index].get('precontact_candidates', [])):
            if candidate[1]=='None':files.append(directory/f'candidate_spheres_{window_index}_{j}.npz')
    arrays = [np.load(p,allow_pickle=False)['spheres'].reshape(-1,4) for p in files if p.exists()]
    return (np.concatenate(arrays) if arrays else None), [p.name for p in files if p.exists()]


def build_exclusions(run_dirs, start=None, end=None, half_size=.025):
    if not run_dirs or not 0 < half_size <= .1 or (start is None)!=(end is None):
        raise ValueError('Invalid exclusion geometry, window, or empty accepted path list')
    result = {'version': 2, 'kind': 'virtual_planner_obstacles',
              'half_size_m': half_size, 'placement': 'arc_length_with_boundary_geometry_filter',
              'sources': [], 'segments': []}
    selected = None
    for index, directory in enumerate(run_dirs):
        directory = Path(directory)
        report = json.loads((directory / 'pipeline_result.json').read_text())
        if not report.get('full_pipeline_success'):
            raise ValueError(f'{directory}: source is not a full successful rollout')
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
                selected=[(w['start'],w['end']) for w in chosen]
            if not selected:raise ValueError('No cuRobo intervals found in accepted rollout')
            result['segments']=[{'window':list(w),'sources':[],'cuboid':{}} for w in selected]
        for segment in result['segments']:
            a,b=segment['window']
            match=[i for i,w in enumerate(windows) if (w['start'],w['end'])==(a,b) and w['controller']=='CUROBO']
            if len(match)!=1:raise ValueError('Accepted paths have incompatible motion intervals')
            path=transport_points(trace,a,b)
            spheres,files=_boundaries(directory,report,match[0])
            boxes,stats=obstacles_along_path(path,half_size,boundary_spheres=spheres)
            segment['sources'].append({**metadata,'wrist_path_xyz':resample(path).tolist(),
                                       'boundary_geometry_files':files,'placement_diagnostics':stats})
            segment['cuboid'].update({f'path_exclusion_{index}_{j}':box for j,box in enumerate(boxes.values())})
    return result


def box_clearance(points, boxes):
    """Signed point distance to the union of axis-aligned exclusion boxes."""
    points = np.asarray(points)
    distances = []
    for box in boxes.values():
        if box['pose'][3:] != [1., 0., 0., 0.]:
            raise ValueError('Only axis-aligned path exclusions are supported')
        delta = np.abs(points - np.asarray(box['pose'][:3])) - np.asarray(box['dims']) / 2
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
        comparisons.append({'source_run': source['run'],
                            'mean_nearest_path_distance_m': float(distances.min(axis=1).mean()),
                            'symmetric_hausdorff_m': float(max(distances.min(axis=1).max(), distances.min(axis=0).max()))})
    return {'actual_wrist_min_exclusion_clearance_m': clearance,
            'actual_wrist_exclusion_pass': clearance > .002,
            'comparisons': comparisons}
