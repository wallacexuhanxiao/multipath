"""Only the scientific invariants of the new V0 path."""
import json
import numpy as np
import pytest
from multipath.oracle_schedule import OracleSchedule
from multipath.planning.timing import fixed_budget_path


def test_reference_state_cannot_be_verified_by_copying_physical_flag():
    from multipath.anchors.artifact import reachable_entry
    state=dict(physically_acquired_state=True,entry_provenance={'trace':'actual_trace.json'},time_s=1.)
    assert not reachable_entry(dict(state,entry_origin='reference_constructed_state'))
    assert not reachable_entry(dict(state,entry_origin='initial_state'))
    assert reachable_entry(dict(state,entry_origin='predecessor_rollout'))
    assert reachable_entry(dict(state,entry_origin='physically_acquired_snapshot'))


def test_core_context_and_immutable_schedule(tmp_path):
    source=json.loads(open('configs/oracle/put_in.json').read())
    path=tmp_path/'schedule.json';path.write_text(json.dumps(source))
    schedule=OracleSchedule.load(path)
    assert [(a.core_start,a.core_end) for a in schedule.anchors.values()]==[(35,76),(82,101),(111,128)]
    assert all(a.synth_start<a.core_start and a.synth_end>a.core_end for a in schedule.anchors.values())
    source['windows'][2]['contact']='free'
    path.write_text(json.dumps(source))
    with pytest.raises(ValueError,match='contact mode'):OracleSchedule.load(path)


def test_cannot_stretch_source_time():
    with pytest.raises(RuntimeError,match='violates dynamics'):
        fixed_budget_path(np.tile(np.linspace(0,1,121)[:,None],(1,7)),1/120,24,1/120,np.ones(7)*3,np.ones(7)*15)
    # A planner's unnecessarily long stationary proposal is compressible.
    path,_=fixed_budget_path(np.zeros((121,7)),1/120,24,1/120,np.ones(7)*3,np.ones(7)*15)
    assert len(path)==25


def test_retiming_checks_acceleration_and_preserves_tick_count():
    raw=np.zeros((25,7))
    path,report=fixed_budget_path(raw,1/120,36,1/120,np.ones(7)*3,np.ones(7)*15)
    assert len(path)-1==36 and report['budget_s']==.3
    raw[12:,0]=.1
    with pytest.raises(RuntimeError,match='violates dynamics'):
        fixed_budget_path(raw,1/120,24,1/120,np.ones(7)*3,np.ones(7)*15)


def test_artifact_rejects_wrong_source_clock_and_nonfinite_commands(tmp_path):
    from multipath.anchors.artifact import save_anchor,load_validated_anchor
    from multipath.anchors.schema import InteractionAnchor
    anchor=InteractionAnchor('grasp',2,4,0,6,'free','grasped')
    entry={'time_s':2/30,'physically_acquired_state':True,'entry_origin':'predecessor_rollout',
           'entry_provenance':{'trace':'physical.json'},'contact':{'mode':'free'}}
    exit_state=dict(entry,time_s=4/30,contact={'mode':'grasped'})
    validation=dict(success=True,handoff_verified=True,padded_rollout_complete=True,core_replay_success=True,
                    state_resets_within_padded_rollout=0,state_resets_within_core_replay=0,control_dt=1/120,source_fps=30)
    save_anchor(tmp_path,anchor,entry,exit_state,np.zeros((8,26)),validation,{'source':'one'})
    load_validated_anchor(tmp_path,anchor,provenance={'source':'one'},fps=30,dt=1/120)
    with pytest.raises(ValueError,match='provenance'):
        load_validated_anchor(tmp_path,anchor,provenance={'source':'two'})
    with pytest.raises(ValueError,match='timestep'):
        load_validated_anchor(tmp_path,anchor,dt=1/60)
    commands=np.zeros((8,26));commands[0,0]=np.nan
    np.savez_compressed(tmp_path/'grasp'/'commands.npz',commands=commands)
    with pytest.raises(ValueError):load_validated_anchor(tmp_path,anchor)


def test_initial_anchor_development_can_use_actual_initial_state(tmp_path):
    from multipath.anchors.development import predecessor_snapshot
    state={'source_frame':0,'time_s':0.,'physically_acquired_state':True,'state_origin':'continuous_PhysX_rollout'}
    (tmp_path/'actual_trace.json').write_text(json.dumps([state]))
    (tmp_path/'pipeline_result.json').write_text(json.dumps({'source_provenance':{'source':'one'},'segments':[]}))
    assert predecessor_snapshot(tmp_path,0,{'source':'one'})['entry_origin']=='initial_state'
