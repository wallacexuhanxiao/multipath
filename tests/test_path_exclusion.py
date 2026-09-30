import json
import numpy as np
import pytest
from multipath.path_exclusion import build_exclusions, compare_path, resample, obstacles_along_path


def accepted_run(tmp_path, name, offset=0, success=True):
    out=tmp_path/name;out.mkdir()
    report={'full_pipeline_success':success,'reference_semantics_validated':True,'source_sha256':'same-reference',
            'mount_xyzw':[0,0,0,1],'scene_translation_xyz':[0,0,0],
            'windows':[{'controller':'CUROBO','start':65,'end':86}]}
    (out/'pipeline_result.json').write_text(json.dumps(report))
    trace=[{'mode':'CUROBO','source_frame':65+21*i/20,
            'wrist_xyz_wxyz':[i/20,offset,0,1,0,0,0]} for i in range(21)]
    (out/'actual_trace.json').write_text(json.dumps(trace))
    return out


def test_cumulative_exclusions_preserve_boundary_space_and_block_old_paths(tmp_path):
    a=accepted_run(tmp_path,'a');b=accepted_run(tmp_path,'b',.2)
    config=build_exclusions([a,b])
    assert len(config['sources'])==2
    config=config['segments'][0]
    for box in config['cuboid'].values():
        assert .075<box['pose'][0]<.925
    assert not compare_path(np.array([[0,0,0],[1,0,0]]),config)['actual_wrist_exclusion_pass']
    assert compare_path(np.array([[0,0,0],[0,0,.2],[1,0,.2],[1,0,0]]),config)['actual_wrist_exclusion_pass']


def test_failed_or_scene_mismatched_rollouts_cannot_define_solutions(tmp_path):
    failed=accepted_run(tmp_path,'failed',success=False)
    with pytest.raises(ValueError,match='not a full successful'):
        build_exclusions([failed])
    a=accepted_run(tmp_path,'a');b=accepted_run(tmp_path,'b')
    p=b/'pipeline_result.json';d=json.loads(p.read_text());d['source_sha256']='other';p.write_text(json.dumps(d))
    with pytest.raises(ValueError,match='different scenes'):
        build_exclusions([a,b])


def test_old_placement_success_does_not_pass_corrected_video_contract(tmp_path):
    run=accepted_run(tmp_path,'legacy')
    path=run/'pipeline_result.json';report=json.loads(path.read_text())
    del report['reference_semantics_validated'];path.write_text(json.dumps(report))
    with pytest.raises(ValueError,match='semantics and timing'):
        build_exclusions([run])


def test_path_comparison_is_independent_of_timing_and_duplicate_samples(tmp_path):
    a=accepted_run(tmp_path,'a');config=build_exclusions([a])['segments'][0]
    points=np.array([[0,0,0],[0,0,0],[.01,0,0],[.9,0,0],[1,0,0]])
    np.testing.assert_allclose(resample(points),resample(np.array([[0,0,0],[1,0,0]])))
    comparison=compare_path(points,config)
    assert comparison['comparisons'][0]['symmetric_hausdorff_m']<1e-10
    assert comparison['actual_wrist_min_exclusion_clearance_m']<0


def test_generic_curved_path_translation_scale_and_boundary_geometry():
    angle=np.linspace(0,np.pi,100)
    path=np.c_[np.cos(angle),np.sin(angle),np.zeros(100)]
    boxes,stats=obstacles_along_path(path,half_size=.04)
    shift=np.array([7.,-3.,2.])
    moved,_=obstacles_along_path(2*path+shift,half_size=.08,clearance=.01)
    centers=np.array([x['pose'][:3] for x in boxes.values()])
    moved_centers=np.array([x['pose'][:3] for x in moved.values()])
    np.testing.assert_allclose(moved_centers,2*centers+shift,atol=1e-12)
    protected=np.array([[0,1,0,.3]])
    filtered,more=obstacles_along_path(path,half_size=.04,boundary_spheres=protected)
    assert len(filtered)<len(boxes)
    assert more['boundary_filtered_count']>stats['boundary_filtered_count']
    with pytest.raises(ValueError,match='No room'):
        obstacles_along_path(np.array([[0,0,0],[.02,0,0]]))


def test_window_discovery_does_not_depend_on_putin_frame_numbers(tmp_path):
    run=accepted_run(tmp_path,'other')
    p=run/'pipeline_result.json';report=json.loads(p.read_text())
    report['windows']=[{'controller':'BC','start':0,'end':700},
                       {'controller':'CUROBO','start':700,'end':900},
                       {'controller':'BC','start':900,'end':1000}]
    p.write_text(json.dumps(report));p=run/'actual_trace.json';trace=json.loads(p.read_text())
    for i,r in enumerate(trace):r['source_frame']=700+200*i/(len(trace)-1)
    p.write_text(json.dumps(trace))
    assert build_exclusions([run])['segments'][0]['window']==[700,900]


def test_serial_search_accumulates_only_successful_candidates(tmp_path,monkeypatch):
    import importlib.util
    import sys
    from pathlib import Path
    from types import SimpleNamespace
    spec=importlib.util.spec_from_file_location('explore_diverse_paths',Path(__file__).parents[1]/'scripts/explore_diverse_paths.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    baseline=accepted_run(tmp_path,'baseline');seen=[]
    def run(command,**kwargs):
        exclusion_path=Path(command[command.index('--path-exclusions')+1])
        seen.append(len(json.loads(exclusion_path.read_text())['sources']))
        out=Path(command[command.index('--output-dir')+1])
        report=json.loads((baseline/'pipeline_result.json').read_text())
        passed=len(seen)>1
        report.update(full_pipeline_success=passed,diverse_solution_success=passed,task_success=passed)
        (out/'pipeline_result.json').write_text(json.dumps(report))
        (out/'actual_trace.json').write_text((baseline/'actual_trace.json').read_text())
        return SimpleNamespace(returncode=0 if passed else 1)
    monkeypatch.setattr(module.subprocess,'run',run)
    monkeypatch.setattr(sys,'argv',['explore','--accepted-run',str(baseline),'--additional-solutions','2',
                                    '--output-root',str(tmp_path/'search'),'--','--reference','unused'])
    module.main()
    manifest=json.loads((tmp_path/'search/solutions.json').read_text())
    assert seen==[1,1,2]
    assert manifest['status']=='complete' and len(manifest['accepted_runs'])==3
    assert [x['status'] for x in manifest['candidates']]==['rejected','accepted','accepted']


def test_tapered_sizes_are_unimodal_protect_geometry_and_scale():
    from multipath.path_exclusion import tapered_obstacles_along_path
    path=np.array([[0.,0,0],[.5,.1,0],[1.,0,0]])
    protected=np.array([[0.,0,0,.08],[1.,0,0,.08]])
    boxes,stats=tapered_obstacles_along_path(path,.04,protected)
    sizes=np.array(stats['half_sizes_m']);peak=sizes.argmax()
    assert peak>0 and peak<len(sizes)-1
    assert np.all(np.diff(sizes[:peak+1])>=-1e-12)
    assert np.all(np.diff(sizes[peak:])<=1e-12)
    assert sizes[0]<sizes[peak] and sizes[-1]<sizes[peak]
    assert stats['minimum_boundary_clearance_m']>.005
    shift=np.array([4.,-2,1])
    scaled,s2=tapered_obstacles_along_path(2*path+shift,.08,
                                          np.c_[2*protected[:,:3]+shift,2*protected[:,3]],.01)
    np.testing.assert_allclose(s2['half_sizes_m'],2*sizes,atol=1e-10)
    np.testing.assert_allclose([b['pose'][:3] for b in scaled.values()],
                               2*np.array([b['pose'][:3] for b in boxes.values()])+shift,atol=1e-10)


def test_wrist_detour_candidates_avoid_tapered_obstacles_and_translate():
    from multipath.path_exclusion import tapered_obstacles_along_path,box_clearance
    from multipath.diverse_planning import detour_waypoints
    path=np.array([[0.,0,0],[.3,0,0]])
    boxes,_=tapered_obstacles_along_path(path)
    config={'cuboid':boxes,'sources':[{'wrist_path_xyz':resample(path).tolist()}]}
    candidates=list(detour_waypoints(*path,config))
    assert candidates and all(box_clearance(p,boxes).min()>.008 for p in candidates)
    shift=np.array([.5,-2.,1.])
    moved={key:{'dims':b['dims'],'pose':[*list(np.array(b['pose'][:3])+shift),1.,0.,0.,0.]} for key,b in boxes.items()}
    c2={'cuboid':moved,'sources':[{'wrist_path_xyz':(resample(path)+shift).tolist()}]}
    translated=list(detour_waypoints(*(path+shift),c2))
    np.testing.assert_allclose(translated,np.array(candidates)+shift,atol=1e-12)


def test_longest_budget_policy_selects_by_duration_not_frame_ids(tmp_path):
    run=accepted_run(tmp_path,'longest')
    p=run/'pipeline_result.json';r=json.loads(p.read_text())
    r['windows']=[{'controller':'BC','start':0,'end':10},
                  {'controller':'CUROBO','start':10,'end':30},
                  {'controller':'BC','start':30,'end':60},
                  {'controller':'CUROBO','start':60,'end':100},
                  {'controller':'BC','start':100,'end':110}]
    p.write_text(json.dumps(r))
    p=run/'actual_trace.json';trace=json.loads(p.read_text())
    for i,item in enumerate(trace):item['source_frame']=60+2*i
    p.write_text(json.dumps(trace))
    config=build_exclusions([run],adaptive_size=True,exclusion_scope='wrist',window_policy='longest-budget')
    assert [s['window'] for s in config['segments']]==[[60,100]]
    assert config['segments'][0]['boundary_wrist_xyz_wxyz']==trace[-1]['wrist_xyz_wxyz']


def test_motion_budget_resampling_ignores_waypoint_stop_duplicates():
    from multipath.diverse_planning import resample_with_velocity_budget
    uneven=np.r_[np.zeros((100,2)),[[.1,.2],[.2,.4]],np.repeat([[.2,.4]],200,axis=0)]
    compact=np.array([[0.,0.],[.1,.2],[.2,.4]])
    np.testing.assert_allclose(resample_with_velocity_budget(uneven,25,[1.,2.]),
                               resample_with_velocity_budget(compact,25,[1.,2.]))


def test_fixed_budget_easing_reduces_endpoint_speed_without_overspeed():
    from multipath.diverse_planning import resample_with_velocity_budget
    path=np.array([[0.,0.],[.2,.4]])
    out=resample_with_velocity_budget(path,41,[1.,2.],dt=.01)
    velocity=np.abs(np.diff(out,axis=0))/.01
    assert np.max(velocity/np.array([1.,2.]))<=1+1e-8
    assert np.max(velocity[0])<np.max(velocity[len(velocity)//2])*.1
    np.testing.assert_allclose(out[[0,-1]],path)
    with pytest.raises(RuntimeError,match='usable budget'):
        resample_with_velocity_budget(path,10,[1.,2.],dt=.01)
