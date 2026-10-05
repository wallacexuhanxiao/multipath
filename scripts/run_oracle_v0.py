"""V0 entrypoint: no VLM, no complete robot reference, no diversity."""
import argparse
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--proposals',type=Path,required=True)
    p.add_argument('--schedule',type=Path,required=True)
    p.add_argument('--validated-anchors',type=Path,help='Final rollout: consume physically validated local Anchor artifacts')
    p.add_argument('--continue-development',action='store_true',help='Use available validated Anchors and develop missing cores from actual predecessor execution')
    p.add_argument('--stop-before-anchor',help='Acquire a continuous predecessor prefix before this core')
    p.add_argument('--through-anchor',help='Stop after this Anchor for an early continuous connection experiment')
    p.add_argument('--embodiment',choices=['mounted','floating_xhand'],default='mounted')
    p.add_argument('--floating-config',type=Path)
    p.add_argument('--robot-yaml',type=Path)
    p.add_argument('--xhand-urdf',type=Path,required=True)
    p.add_argument('--object-cache',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    args=p.parse_args();out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    if args.embodiment=='mounted' and not args.robot_yaml:p.error('mounted requires --robot-yaml')
    if args.embodiment=='floating_xhand' and not args.floating_config:p.error('floating_xhand requires --floating-config')
    if args.embodiment=='floating_xhand' and (args.continue_development or not (args.validated_anchors or args.stop_before_anchor)):
        p.error('floating continuous execution requires validated Anchors; synthesize them locally first')
    from multipath.oracle_schedule import OracleSchedule
    schedule=OracleSchedule.load(args.schedule)
    if (args.embodiment=='floating_xhand' and not args.validated_anchors and
        args.stop_before_anchor!=min(schedule.anchors.values(),key=lambda a:a.core_start).id):
        p.error('Later floating prefixes require validated predecessor Anchors')
    if args.through_anchor and args.through_anchor not in schedule.anchors:raise ValueError('Unknown stop Anchor')
    if args.stop_before_anchor and args.stop_before_anchor not in schedule.anchors:raise ValueError('Unknown prefix boundary')
    if args.stop_before_anchor and args.through_anchor:raise ValueError('Choose one prefix boundary')
    manifest=json.loads((args.proposals/'manifest.json').read_text())
    import hashlib
    if manifest['provenance']['schedule_sha256']!=hashlib.sha256(args.schedule.read_bytes()).hexdigest():
        raise ValueError('Local proposals and oracle schedule differ')
    if args.validated_anchors:
        from multipath.anchors.artifact import load_validated_anchor
        for anchor in schedule.anchors.values():
            if args.stop_before_anchor and anchor.core_start>=schedule.anchors[args.stop_before_anchor].core_start:continue
            if args.through_anchor and anchor.core_start>schedule.anchors[args.through_anchor].core_start:continue
            if args.continue_development and not (args.validated_anchors/anchor.id/'report.json').exists():continue
            artifact=load_validated_anchor(args.validated_anchors,anchor)
            if artifact['report']['source_provenance']!=manifest['provenance']:
                raise ValueError('Canonical Anchor and local inputs have different provenance')
            if artifact['entry'].get('embodiment','mounted')!=args.embodiment:
                raise ValueError('Canonical Anchor embodiment differs from runtime')
    import shutil
    implementation=out/'implementation'
    implementation.mkdir(exist_ok=True)
    shutil.copy2(__file__,implementation/'run_oracle_v0.py')
    import multipath
    shutil.copytree(Path(multipath.__file__).parent,implementation/'multipath',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
    from dataclasses import asdict
    for anchor in schedule.anchors.values():
        folder=out/'anchors'/anchor.id;folder.mkdir(parents=True,exist_ok=True)
        pending=dict(asdict(anchor),core_interval=[anchor.core_start,anchor.core_end],synthesis_interval=[anchor.synth_start,anchor.synth_end],
                     entry_physically_reachable=None,local_rollout_success=False,handoff_verified=False,contact_state=None,
                     source_provenance=manifest['provenance'],status='not_executed_dependency_pending')
        (folder/'report.json').write_text(json.dumps(pending,indent=2))
    app=None;scene=None
    report={'status':'initializing','full_pipeline_success':False,'task_success':False,
            'pbs_job_id':__import__('os').environ.get('PBS_JOBID'),'source_provenance':manifest['provenance'],
            'source_sha256':manifest['provenance']['schedule_sha256']}
    try:
        from multipath.execution.startup import start_simulation
        app=start_simulation(out)
        if args.embodiment=='floating_xhand':
            import numpy as np
            from multipath.execution.floating_scene import FloatingScene
            from multipath.execution.floating_executor import execute
            first=min(schedule.anchors.values(),key=lambda a:a.core_start)
            data=dict(np.load(args.proposals/manifest['anchors'][first.id],allow_pickle=False))
            initial=data['wrist_hand_q'][0,:7].copy()
            initial[2]+=max(.1,float(np.linalg.norm(np.ptp(data['human_joints'][0],axis=0))))
            scene=FloatingScene(args,manifest,out,initial,initial_hand=data['wrist_hand_q'][0,7:])
            report.update(embodiment=scene.embodiment,assembly_self_collision=False)
        else:
            from multipath.execution.mounted_scene import MountedScene
            from multipath.execution.schedule_executor import execute
            scene=MountedScene(args,manifest,out)
        report.update(mount_xyzw=manifest['mount_xyzw'],scene_translation_xyz=[0,0,0],physics='PhysX CPU',
                      initial_state=scene.state(),initial_ready_search=getattr(scene,'ready_search',None),table_z=float(scene.table['pose'][2]+.02))
        execute(scene,schedule,manifest,args.proposals,report)
    except Exception as error:
        import traceback;traceback.print_exc()
        report.update(status='failed',error=f'{type(error).__name__}: {error}',full_pipeline_success=False)
    finally:
        if scene:
            try:
                if not scene.records or scene.records[-1]['time_s']<scene.ticks*scene.dt-1e-8:
                    scene.capture('STOP',scene.ticks*scene.dt*schedule.fps)
            except Exception as error:
                report['capture_error']=str(error)
            try:scene.save()
            except Exception as error:report['trace_error']=str(error)
            if 'capture_error' in report or 'trace_error' in report:
                report.update(status='recording_failed',full_pipeline_success=False,task_success=False,
                              prefix_success=False,development_success=False)
            report['actual_duration_s']=scene.ticks*scene.dt
            report['frames_recorded']=len(scene.records)
            if args.embodiment=='mounted':
                report['measured_velocity_ratio']=scene.max_velocity_ratio
                report['measured_acceleration_ratio']=scene.max_acceleration_ratio
        (out/'pipeline_result.json').write_text(json.dumps(report,indent=2))
        print('ORACLE_RESULT',report['status'],report.get('error',''),flush=True)
        if app:app.close()
    if not (report['full_pipeline_success'] or report.get('prefix_success') or report.get('development_success')):raise SystemExit(1)

if __name__=='__main__':main()
