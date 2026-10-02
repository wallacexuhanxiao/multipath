"""Development only: re-execute historical controls to measure missing snapshots.

Old poses/finite-difference velocities are never relabeled as measured states.
This is not the Oracle runtime and establishes no task or Anchor success.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser()
    for name in ('proposals','schedule','command-trace','robot-yaml','xhand-urdf','object-cache','output-dir'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--anchors',nargs='+',required=True)
    args=p.parse_args();out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    from multipath.oracle_schedule import OracleSchedule
    schedule=OracleSchedule.load(args.schedule)
    frames=sorted({schedule.anchors[a].synth_start for a in args.anchors})
    manifest=json.loads((args.proposals/'manifest.json').read_text())
    if manifest['provenance']['schedule_sha256']!=hashlib.sha256(args.schedule.read_bytes()).hexdigest():
        raise ValueError('Acquisition proposals and schedule differ')
    old=json.loads(args.command_trace.read_text())
    source_frames=np.array([r['source_frame'] for r in old])
    if frames[-1]>source_frames[-1]:raise ValueError('Historical controls do not cover requested context entries')
    # The historical mounted command schema is arm7 followed by XHand12.
    q=np.array([r['command'] for r in old])
    if q.shape[1]!=19:raise ValueError('Historical command joint layout must be arm7 + XHand12')
    query=np.arange(1,frames[-1]+1)
    targets=np.stack([np.interp(query,source_frames,q[:,j]) for j in range(19)],axis=1)
    app=None;scene=None
    report={'source':'oracle','source_fps':schedule.fps,'source_provenance':manifest['provenance'],
            'purpose':'development_snapshot_acquisition','full_pipeline_success':False,'task_success':False,
            'state_resets_between_segments':0,'historical_control_trace':str(args.command_trace),
            'historical_control_sha256':hashlib.sha256(args.command_trace.read_bytes()).hexdigest(),
            'snapshots':{},'segments':[],'status':'initializing'}
    try:
        from multipath.execution.startup import start_simulation
        app=start_simulation(out)
        from multipath.execution.mounted_scene import MountedScene
        from multipath.anchors.synthesis import fit_timed_controls,hand_limits_from_urdf
        scene=MountedScene(args,manifest,out);start=scene.state()
        limits=np.asarray(scene.robot._articulation_view.get_dof_limits()).reshape(-1,2)[scene.arm]
        controls,fitting=fit_timed_controls(targets[:,:7],targets[:,7:],np.array(start['arm_q']),
            np.array(start['arm_dq']),np.array(start['hand_q']),schedule.fps,scene.dt,scene.velocity,scene.acceleration,limits,
            start_hand_dq=np.array(start['hand_dq']),hand_velocity_limits=hand_limits_from_urdf(scene),
            hand_startup_duration=min((a.core_start-a.synth_start)/schedule.fps for a in schedule.anchors.values()),
            hand_joint_limits=np.asarray(scene.robot._articulation_view.get_dof_limits()).reshape(-1,2)[scene.fingers])
        report['control_fitting']=fitting
        np.savez_compressed(out/'acquisition_controls.npz',commands=controls)
        tpf=round(1/(schedule.fps*scene.dt));previous=0
        for k,cmd in enumerate(controls):
            frame=(k+1)/tpf
            scene.step(cmd[:7],cmd[7:19],'SNAPSHOT_ACQUISITION',frame,cmd[19:26])
            if frame in frames:
                state=scene.state();state['entry_origin']='predecessor_rollout'
                state['entry_provenance']={'trace':str(out/'actual_trace.json'),'source_frame':frame,
                                           'acquisition_controls':str(out/'acquisition_controls.npz')}
                (out/f'snapshot_{int(frame)}.json').write_text(json.dumps(state,indent=2))
                report['snapshots'][str(int(frame))]={'physically_acquired_state':True,'contact':state['contact']}
                report['segments'].append({'start':previous,'end':frame,'success':True,'mode':'PHYSICAL_ACQUISITION'})
                previous=frame
        report['status']='acquired'
    except Exception as error:
        import traceback;traceback.print_exc()
        report.update(status='failed',error=str(error))
    finally:
        if scene:
            try:
                if not scene.records or scene.records[-1]['time_s']<scene.ticks*scene.dt:scene.capture('STOP',scene.ticks*scene.dt*schedule.fps)
            except Exception as error:report.update(status='failed',capture_error=str(error))
            try:scene.save()
            except Exception as error:report.update(status='failed',trace_error=str(error))
            report.update(actual_duration_s=scene.ticks*scene.dt,table_z=float(scene.table['pose'][2]+.02))
        (out/'pipeline_result.json').write_text(json.dumps(report,indent=2))
        if app:app.close()
    if report['status']!='acquired':raise SystemExit(1)


if __name__=='__main__':main()
