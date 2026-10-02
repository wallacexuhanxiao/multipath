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


def save_anchor(root, anchor, entry, exit_state, commands, validation, provenance):
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
    np.savez_compressed(root/'commands.npz', commands=np.asarray(commands))
    return report


def load_validated_anchor(root, anchor):
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
    commands=np.load(root/'commands.npz',allow_pickle=False)['commands']
    dt=report['validation']['control_dt'];fps=report['validation']['source_fps']
    if len(commands)!=round((anchor.core_end-anchor.core_start)/(fps*dt)) or commands.shape[1]!=26:
        raise ValueError('Anchor commands do not match source duration')
    return dict(report=report,entry=entry,exit=exit_state,commands=commands)
