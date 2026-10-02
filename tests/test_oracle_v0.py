"""Only the scientific invariants of the new V0 path."""
import json
import numpy as np
import pytest
from multipath.oracle_schedule import OracleSchedule
from multipath.planning.timing import fixed_budget_path


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
