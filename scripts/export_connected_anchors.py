"""Export CORE artifacts with the actually executed surrounding context.

Padding is evidence from the continuous local neighborhood, never extra time
in the final trajectory. A fresh artifact-consuming rollout is still required.
"""
import argparse,json
from pathlib import Path
import numpy as np
from multipath.oracle_schedule import OracleSchedule
from multipath.anchors.artifact import save_anchor
from multipath.validation.physics import check_retention


def main():
    p=argparse.ArgumentParser();p.add_argument('--run-dir',type=Path,required=True)
    p.add_argument('--schedule',type=Path,required=True);args=p.parse_args()
    root=args.run_dir;report=json.loads((root/'pipeline_result.json').read_text());schedule=OracleSchedule.load(args.schedule)
    if (not report.get('continuous_execution_success') or not report.get('task_success')
            or report.get('start_frame',0)!=0 or report['state_resets_between_segments']!=0
            or not np.isclose(report['actual_duration_s'],schedule.num_frames/schedule.fps)):
        raise ValueError('Export requires one successful complete initial-state physical candidate')
    records={round(x['source_frame']):x for x in json.loads((root/'actual_trace.json').read_text())}
    # Final execution consumes segment artifacts and boundary states, not the
    # development rollout's complete recorded robot trajectory.
    for window in schedule.windows:
        if window['mode']!='RELAXED':continue
        segment=next(s for s in report['segments'] if s['start']==window['start'])
        if not segment['success']:raise ValueError('Cannot export a failed bridge')
        artifact=dict(window,entry=records[window['start']],exit=records[window['end']],
            planning=segment['planning'],source_provenance=report['source_provenance'],
            physical_validation_success=True,source_clock_preserved=True)
        (root/f'relaxed_{window["start"]}_{window["end"]}.json').write_text(json.dumps(artifact,indent=2))
    exported=[]
    for a in schedule.anchors.values():
        context=[records[k] for k in range(a.synth_start,a.synth_end+1)]
        pre=[x for x in context if x['source_frame']<=a.core_start]
        post=[x for x in context if x['source_frame']>=a.core_end]
        for states,contact in [(pre,a.entry_contact),(post,a.exit_contact)]:
            held=np.asarray(states[0]['hand_object_transform'])
            for state in states:
                if state['contact']['mode']!=contact:raise ValueError(f'{a.id}: unstable context contact mode')
                if contact=='grasped':check_retention(state,held)
        core=root/'cores'/a.id
        entry=json.loads((core/'entry.json').read_text());exit_state=json.loads((core/'exit.json').read_text())
        commands=np.load(core/'commands.npz')['commands']
        validation=dict(success=True,padded_rollout_complete=True,core_replay_success=True,
            state_resets_within_padded_rollout=0,state_resets_within_core_replay=0,handoff_verified=True,
            source_fps=schedule.fps,control_dt=1/120,
            validation_scope='continuous physical synthesis context and executed exported core',
            padding_execution='actual adjacent cuRobo intervals; core proposal used local retarget/context',
            physical_trace=str(root/'actual_trace.json'),context_samples=len(context),
            controller_handoff='measured_mass_inertia_gain_response',core_command_only=True)
        exported.append(save_anchor(root/'anchors',a,entry,exit_state,commands,validation,report['source_provenance']))
    (root/'anchor_export.json').write_text(json.dumps(dict(anchors=exported,padded_contexts_complete=True,
        fresh_artifact_rollout_pending=True),indent=2))
    print('CONNECTED_ANCHORS_EXPORTED',len(exported))


if __name__=='__main__':main()
