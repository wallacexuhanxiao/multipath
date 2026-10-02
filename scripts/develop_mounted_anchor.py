"""M2 only: acquired predecessor -> padded replay -> mounted physical MPC."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil


def main():
    p=argparse.ArgumentParser()
    for name in ('proposals','schedule','predecessor-dir','robot-yaml','xhand-urdf','object-cache','output-dir'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--anchor',help='Default: first oracle Anchor')
    p.add_argument('--artifact-dir',type=Path,help='Publish each Anchor here only after padded and exported-core execution pass')
    args=p.parse_args();args.validated_anchors=None
    out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    from multipath.oracle_schedule import OracleSchedule
    schedule=OracleSchedule.load(args.schedule)
    anchor=schedule.anchors[args.anchor] if args.anchor else min(schedule.anchors.values(),key=lambda a:a.core_start)
    manifest=json.loads((args.proposals/'manifest.json').read_text())
    if manifest['provenance']['schedule_sha256']!=hashlib.sha256(args.schedule.read_bytes()).hexdigest():
        raise ValueError('Proposal/schedule mismatch')
    import multipath
    implementation=out/'implementation';implementation.mkdir(exist_ok=True)
    shutil.copy2(__file__,implementation/Path(__file__).name)
    shutil.copytree(Path(multipath.__file__).parent,implementation/'multipath',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
    app=None;scene=None
    report={'milestone':'M2','status':'initializing','anchor':anchor.id,'m2_success':False,
            'full_pipeline_success':False,'pbs_job_id':os.environ.get('PBS_JOBID'),
            'source_provenance':manifest['provenance'],'bridge_backend_changed':False,
            'snapshot_restore_scope':'local_development_only','rl_status':'not_attempted',
            'core_interval':[anchor.core_start,anchor.core_end],
            'synthesis_interval':[anchor.synth_start,anchor.synth_end]}
    try:
        import numpy as np
        from multipath.anchors.development import DevelopmentScene,predecessor_snapshot,replay_padded,validate_exported_core,mounted_mpc
        from multipath.anchors.synthesis import local_commands,fit_timed_controls,hand_limits_from_urdf
        start=predecessor_snapshot(args.predecessor_dir,anchor.synth_start,manifest['provenance'])
        (out/'acquired_snapshot.json').write_text(json.dumps(start,indent=2))
        from multipath.execution.startup import start_simulation
        app=start_simulation(out)
        scene=DevelopmentScene(args,manifest,out)
        scene.restore_development(start)
        data=dict(np.load(args.proposals/manifest['anchors'][anchor.id],allow_pickle=False))
        # Smooth the measured-to-local-target difference over PRE context only.
        offset=np.array(start['wrist_xyz_wxyz'][:3])-data['wrist_hand_q'][0,:3]
        held=np.array(start['hand_object_transform']) if start['contact']['mode']=='grasped' else None
        targets=local_commands(scene,anchor,data,held,offset,padded=True)
        limits=np.asarray(scene.robot._articulation_view.get_dof_limits()).reshape(-1,2)[scene.arm]
        controls,fitting=fit_timed_controls(targets[:,:7],targets[:,7:],np.array(start['arm_q']),
             np.array(start['arm_dq']),np.array(start['hand_q']),schedule.fps,scene.dt,scene.velocity,scene.acceleration,limits,
             start_hand_dq=np.array(start['hand_dq']),hand_velocity_limits=hand_limits_from_urdf(scene),hand_startup_duration=(anchor.core_start-anchor.synth_start)/schedule.fps,
             hand_joint_limits=np.asarray(scene.robot._articulation_view.get_dof_limits()).reshape(-1,2)[scene.fingers])
        report['kinematic_initialization']=fitting
        from multipath.anchors.control_check import control_feasibility
        all_limits=np.asarray(scene.robot._articulation_view.get_dof_limits()).reshape(-1,2)[np.r_[scene.arm,scene.fingers]]
        precheck=control_feasibility(controls,np.array(start['arm_q']),np.array(start['arm_dq']),scene.dt,all_limits,scene.velocity,scene.acceleration,
            start_hand_q=np.array(start['hand_q']),hand_velocity_limits=hand_limits_from_urdf(scene))
        report['nominal_precheck']=precheck
        if not precheck['feasible']:raise RuntimeError('Local fitted replay failed nominal precheck')
        print('PADDED_REPLAY_BEGIN',anchor.id,flush=True)
        result=replay_padded(scene,anchor,data,controls,start,out/'replay','mounted_padded_replay')
        report['padded_replay']=result
        if result['success']:
            result=validate_exported_core(scene,anchor,data,out/'replay',out/'replay_core',args.artifact_dir)
        report['replay']=result
        report['status']='replay_passed' if result['success'] else 'replay_failed'
        (out/'development_result.json').write_text(json.dumps(report,indent=2))
        if not result['success']:
            print('MOUNTED_MPC_BEGIN',result['failures'],flush=True)
            refined=mounted_mpc(scene,anchor,data,controls,start,out/'mpc_search')
            result=replay_padded(scene,anchor,data,refined,start,out/'mpc_replay','mounted_PhysX_shooting_MPC')
            report['padded_mpc_replay']=result
            if result['success']:
                result=validate_exported_core(scene,anchor,data,out/'mpc_replay',out/'mpc_core',args.artifact_dir)
            report['mpc']=result
            report['status']='mpc_passed' if result['success'] else 'mounted_mpc_failed'
            if not result['success']:report['rl_status']='not_attempted; review interaction failure first'
        report['m2_success']=bool(result['success'] and result.get('core_replay_success'))
        if report['m2_success']:
            report['validated_artifact_dir']=str(args.artifact_dir or scene.out/'anchors')
    except Exception as error:
        import traceback;traceback.print_exc()
        report.update(status='failed',error=f'{type(error).__name__}: {error}')
    finally:
        if scene:report['development_snapshot_restores']=getattr(scene,'restore_count',0)
        (out/'development_result.json').write_text(json.dumps(report,indent=2))
        print('M2_RESULT',report['status'],report.get('error',''),flush=True)
        if app:app.close()
    if not report['m2_success']:raise SystemExit(1)


if __name__=='__main__':main()
