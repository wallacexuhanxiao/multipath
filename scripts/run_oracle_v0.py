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
    p.add_argument('--through-anchor',help='Stop after this Anchor for an early continuous connection experiment')
    p.add_argument('--robot-yaml',type=Path,required=True)
    p.add_argument('--xhand-urdf',type=Path,required=True)
    p.add_argument('--object-cache',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    args=p.parse_args();out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    from multipath.oracle_schedule import OracleSchedule
    schedule=OracleSchedule.load(args.schedule)
    if args.through_anchor and args.through_anchor not in schedule.anchors:raise ValueError('Unknown stop Anchor')
    manifest=json.loads((args.proposals/'manifest.json').read_text())
    import hashlib
    if manifest['provenance']['schedule_sha256']!=hashlib.sha256(args.schedule.read_bytes()).hexdigest():
        raise ValueError('Local proposals and oracle schedule differ')
    if args.validated_anchors:
        from multipath.anchors.artifact import load_validated_anchor
        for anchor in schedule.anchors.values():
            if args.through_anchor and anchor.core_start>schedule.anchors[args.through_anchor].core_start:continue
            if args.continue_development and not (args.validated_anchors/anchor.id/'report.json').exists():continue
            artifact=load_validated_anchor(args.validated_anchors,anchor)
            if artifact['report']['source_provenance']!=manifest['provenance']:
                raise ValueError('Canonical Anchor and local inputs have different provenance')
    import shutil
    implementation=out/'implementation'
    implementation.mkdir(exist_ok=True)
    shutil.copy2(__file__,implementation/'run_oracle_v0.py')
    import multipath
    shutil.copytree(Path(multipath.__file__).parent,implementation/'multipath',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
    from isaacsim import SimulationApp
    from dataclasses import asdict
    for anchor in schedule.anchors.values():
        folder=out/'anchors'/anchor.id;folder.mkdir(parents=True,exist_ok=True)
        pending=dict(asdict(anchor),core_interval=[anchor.core_start,anchor.core_end],synthesis_interval=[anchor.synth_start,anchor.synth_end],
                     entry_physically_reachable=None,local_rollout_success=False,handoff_verified=False,contact_state=None,
                     source_provenance=manifest['provenance'],status='not_executed_dependency_pending')
        (folder/'report.json').write_text(json.dumps(pending,indent=2))
    app=SimulationApp({'headless':True});scene=None
    report={'status':'initializing','full_pipeline_success':False,'task_success':False,
            'pbs_job_id':__import__('os').environ.get('PBS_JOBID'),'source_provenance':manifest['provenance'],
            'source_sha256':manifest['provenance']['schedule_sha256']}
    try:
        from multipath.execution.mounted_scene import MountedScene
        from multipath.execution.schedule_executor import execute
        scene=MountedScene(args,manifest,out)
        report.update(mount_xyzw=manifest['mount_xyzw'],scene_translation_xyz=[0,0,0],physics='PhysX CPU',
                      initial_state=scene.state(),initial_ready_search=scene.ready_search,table_z=float(scene.table['pose'][2]+.02))
        execute(scene,schedule,manifest,args.proposals,report)
    except Exception as error:
        import traceback;traceback.print_exc()
        report.update(status='failed',error=f'{type(error).__name__}: {error}',full_pipeline_success=False)
    finally:
        if scene:
            if scene.records[-1]['time_s']<scene.ticks*scene.dt-1e-8:
                scene.capture('STOP',scene.ticks*scene.dt*schedule.fps)
            scene.save();report['actual_duration_s']=scene.ticks*scene.dt
            report['frames_recorded']=len(scene.records)
            report['measured_velocity_ratio']=scene.max_velocity_ratio
            report['measured_acceleration_ratio']=scene.max_acceleration_ratio
        (out/'pipeline_result.json').write_text(json.dumps(report,indent=2))
        print('ORACLE_RESULT',report['status'],report.get('error',''),flush=True)
        app.close()
    if not (report['full_pipeline_success'] or report.get('prefix_success') or report.get('development_success')):raise SystemExit(1)

if __name__=='__main__':main()
