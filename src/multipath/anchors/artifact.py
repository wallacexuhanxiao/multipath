"""Artifacts distinguish local proposals from physically acquired states."""
import json
from dataclasses import asdict
from pathlib import Path
import numpy as np


def reachable_entry(state):
    """A reference pose, even with a copied boolean, is not physical evidence."""
    origin=state.get('entry_origin')
    if not state.get('physically_acquired_state'):
        return False
    if origin=='initial_state':
        return state.get('time_s')==0
    if origin in ('predecessor_rollout','physically_acquired_snapshot'):
        return bool(state.get('entry_provenance'))
    return False


def save_anchor(root, anchor, entry, exit_state, commands, validation, provenance,*,controller_wrenches=None):
    root = Path(root)/anchor.id
    root.mkdir(parents=True, exist_ok=True)
    reachable=reachable_entry(entry)
    success = bool(validation.get('success', False) and reachable
                   and validation.get('padded_rollout_complete')
                   and validation.get('core_replay_success')
                   and validation.get('state_resets_within_padded_rollout') == 0
                   and validation.get('state_resets_within_core_replay') == 0)
    report = dict(asdict(anchor), core_interval=[anchor.core_start, anchor.core_end],
                  synthesis_interval=[anchor.synth_start, anchor.synth_end],
                  entry_physically_reachable=reachable,entry_origin=entry.get('entry_origin'),
                  entry_provenance=entry.get('entry_provenance'),
                  local_rollout_success=success, source_provenance=provenance,
                  contact_state={'entry':entry['contact'], 'exit':exit_state['contact']},
                  handoff_verified=bool(success and validation.get('handoff_verified',False)),
                  validation=validation)
    (root/'report.json').write_text(json.dumps(report, indent=2))
    (root/'entry.json').write_text(json.dumps(entry, indent=2))
    (root/'exit.json').write_text(json.dumps(exit_state, indent=2))
    arrays={'commands':np.asarray(commands)}
    if controller_wrenches is not None:
        wrench=np.asarray(controller_wrenches)
        if wrench.shape!=(len(commands),6) or not np.isfinite(wrench).all():
            raise ValueError('Anchor native controller program has an invalid shape/state')
        arrays['controller_wrenches']=wrench
        report['controller_program']='executed_wrist_finger_twist_plus_native_external_wrench'
        (root/'report.json').write_text(json.dumps(report,indent=2))
    np.savez_compressed(root/'commands.npz', **arrays)
    trace_path=Path(validation.get('physical_trace') or entry.get('entry_provenance',{}).get('trace',''))
    if trace_path.is_file():
        samples=[s for s in json.loads(trace_path.read_text()) if anchor.core_start<=s.get('source_frame',-1)<=anchor.core_end]
        (root/'local_actual_trace.json').write_text(json.dumps(samples))
    return report


def load_validated_anchor(root, anchor, *, provenance=None, fps=None, dt=None):
    root=Path(root)/anchor.id
    report=json.loads((root/'report.json').read_text())
    entry=json.loads((root/'entry.json').read_text())
    exit_state=json.loads((root/'exit.json').read_text())
    if (not report['local_rollout_success'] or not report['handoff_verified'] or
        not reachable_entry(entry) or not exit_state.get('physically_acquired_state') or
        not report['validation'].get('padded_rollout_complete') or
        report['validation'].get('state_resets_within_padded_rollout') != 0 or
        not report['validation'].get('core_replay_success') or
        report['validation'].get('state_resets_within_core_replay') != 0 or
        report['core_interval'] != [anchor.core_start,anchor.core_end] or
        report['synthesis_interval'] != [anchor.synth_start,anchor.synth_end]):
        raise ValueError('Anchor is unvalidated, unreachable, or belongs to another schedule')
    if provenance is not None and report['source_provenance']!=provenance:
        raise ValueError('Anchor source provenance mismatch')
    if fps is not None and not np.isclose(report['validation']['source_fps'],fps,rtol=0,atol=1e-8):
        raise ValueError('Anchor source fps mismatch')
    if dt is not None and not np.isclose(report['validation']['control_dt'],dt,rtol=0,atol=1e-10):
        raise ValueError('Anchor physics timestep mismatch')
    program=np.load(root/'commands.npz',allow_pickle=False)
    commands=program['commands']
    wrenches=program['controller_wrenches'] if 'controller_wrenches' in program else None
    if wrenches is not None and (wrenches.shape!=(len(commands),6) or not np.isfinite(wrenches).all()):
        raise ValueError('Invalid native Anchor controller program')
    dt=report['validation']['control_dt'];fps=report['validation']['source_fps']
    if not (np.isfinite(dt) and dt>0 and np.isfinite(fps) and fps>0):raise ValueError('Invalid Anchor clock')
    if (commands.ndim!=2 or commands.shape[1]!=(25 if entry.get('embodiment')=='floating_xhand' else 26) or not np.isfinite(commands).all() or
        len(commands)!=round((anchor.core_end-anchor.core_start)/(fps*dt))):
        raise ValueError('Anchor commands do not match source duration')
    if (not np.isclose(entry['time_s'],anchor.core_start/fps,rtol=0,atol=1e-8) or
        not np.isclose(exit_state['time_s'],anchor.core_end/fps,rtol=0,atol=1e-8)):
        raise ValueError('Anchor measured boundary clocks mismatch')
    if entry['contact']['mode']!=anchor.entry_contact or exit_state['contact']['mode']!=anchor.exit_contact:
        raise ValueError('Anchor contact boundary mismatch')
    return dict(report=report,entry=entry,exit=exit_state,commands=commands,controller_wrenches=wrenches)
