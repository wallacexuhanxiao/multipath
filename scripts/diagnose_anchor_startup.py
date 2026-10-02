"""Ten-step controller takeover diagnostic; never produces a valid Anchor.

The diagnostic records limit violations rather than stopping at the first one.
All variants start from the same physically acquired snapshot, with no settle.
"""
import argparse,json
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser()
    for name in ('proposals','schedule','snapshot','controls','robot-yaml','xhand-urdf','object-cache','output-dir'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--steps',type=int,default=10)
    p.add_argument('--case',choices=['bounded_hand','bounded_arm_hand'])
    args=p.parse_args();out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((args.proposals/'manifest.json').read_text())
    snapshot=json.loads(args.snapshot.read_text())
    controls=np.load(args.controls)['commands']
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True});report={'diagnostic_only':True,'full_pipeline_success':False,'cases':{}}
    try:
        from multipath.anchors.development import DevelopmentScene
        scene=DevelopmentScene(args,manifest,out)
        order=np.r_[scene.arm,scene.fingers]
        limits=np.asarray(scene.robot._articulation_view.get_dof_limits()).reshape(-1,2)[order]
        joint_names=[scene.robot.dof_names[i] for i in order]
        report['joint_names']=joint_names;report['limits']=limits.tolist()
        gains=scene.robot._articulation_view.get_gains()
        kp=np.asarray(gains[0]).reshape(-1)[order];kd=np.asarray(gains[1]).reshape(-1)[order]
        report['kp']=kp.tolist();report['kd']=kd.tolist()
        base=controls[:48]
        velocity=np.diff(np.vstack((snapshot['arm_q'],base[:,:7])),axis=0)/scene.dt
        accel=np.diff(np.vstack((snapshot['arm_dq'],velocity)),axis=0)/scene.dt
        bad=np.argwhere((base[:,:19]<limits[:,0])|(base[:,:19]>limits[:,1]))
        report['baseline_precheck']={'velocity_ratio':float(np.max(abs(velocity)/scene.velocity)),
          'acceleration_ratio':float(np.max(abs(accel)/scene.acceleration)),
          'joint_limit_violations':[{'tick':int(i),'joint':joint_names[j],'q':float(base[i,j]),'limits':limits[j].tolist()} for i,j in bad[:30]]}
        def read():
            q=np.asarray(scene.robot.get_joint_positions())[order].copy()
            dq=np.asarray(scene.robot.get_joint_velocities())[order].copy()
            view=scene.robot._articulation_view._physics_view
            qt=np.asarray(view.get_dof_position_targets()).reshape(-1)[order]
            vt=np.asarray(view.get_dof_velocity_targets()).reshape(-1)[order]
            return {'q_actual':q.tolist(),'dq_actual':dq.tolist(),'q_target':qt.tolist(),
              'dq_feedforward':vt.tolist(),'position_error':(qt-q).tolist(),
              'pd_effort_estimate':(kp*(qt-q)+kd*(vt-dq)).tolist(),
              'measured_joint_efforts':np.asarray(scene.robot.get_measured_joint_efforts())[order].tolist(),
              'applied_effort_command':np.asarray(scene.robot.get_applied_joint_efforts())[order].tolist()}
        initial=np.r_[snapshot['arm_q'],snapshot['hand_q'],snapshot['arm_dq']]
        from multipath.anchors.synthesis import fit_hand_startup,fit_arm_startup,hand_limits_from_urdf
        from multipath.oracle_schedule import OracleSchedule
        schedule=OracleSchedule.load(args.schedule)
        anchor=next(a for a in schedule.anchors.values() if abs(a.synth_start/schedule.fps-snapshot['time_s'])<1e-8)
        duration=(anchor.core_start-anchor.synth_start)/schedule.fps
        bounded=controls.copy()
        bounded[:,7:19]=fit_hand_startup(controls[:,7:19],np.array(snapshot['hand_q']),np.array(snapshot['hand_dq']),scene.dt,limits[7:],hand_limits_from_urdf(scene),startup_duration=duration)
        report['hand_startup_duration_s']=duration
        np.savez_compressed(out/'bounded_hand_controls.npz',commands=bounded)
        coupled=bounded.copy()
        coupled[:,:7]=fit_arm_startup(controls[:,:7],np.array(snapshot['arm_q']),np.array(snapshot['arm_dq']),scene.dt,scene.velocity,scene.acceleration,limits[:7],startup_duration=duration)
        coupled[:,19:26]=np.diff(np.vstack((snapshot['arm_q'],coupled[:,:7])),axis=0)/scene.dt
        np.savez_compressed(out/'bounded_arm_hand_controls.npz',commands=coupled)
        for name,align,left in [('saved_targets',False,False),('measured_targets',True,False),('measured_left_boundary',True,True),('arm_only_hand_fixed',True,False),('hand_only_arm_constant_velocity',True,False),('bounded_hand',True,False),('bounded_arm_hand',True,False)]:
            if args.case and name!=args.case:continue
            scene.restore_development(snapshot,align_targets=False)
            restored=read()
            if align:
                scene.qcommand=scene.robot.get_joint_positions().copy()
                scene.velocity_command=scene.robot.get_joint_velocities().copy()
                scene.robot.apply_action(scene.Action(joint_positions=scene.qcommand,joint_velocities=scene.velocity_command))
            case={'after_snapshot_restore':restored,'before_first_command':read(),'steps':[]}
            report['cases'][name]=case
            for k in range(args.steps):
                cmd=((initial if k==0 else controls[k-1]) if left else controls[k]).copy()
                if name=='bounded_hand':cmd=bounded[k].copy()
                if name=='bounded_arm_hand':cmd=coupled[k].copy()
                if name=='arm_only_hand_fixed':cmd[7:19]=snapshot['hand_q']
                if name=='hand_only_arm_constant_velocity':
                    cmd[:7]=np.array(snapshot['arm_q'])+(k+1)*scene.dt*np.array(snapshot['arm_dq'])
                    cmd[19:26]=snapshot['arm_dq']
                before=read()
                scene.qcommand[scene.arm]=cmd[:7];scene.qcommand[scene.fingers]=cmd[7:19]
                scene.velocity_command[:]=0;scene.velocity_command[scene.arm]=cmd[19:26]
                scene.robot.apply_action(scene.Action(joint_positions=scene.qcommand,joint_velocities=scene.velocity_command))
                drive=read();scene.world.step(render=False);scene.ticks+=1;after=read()
                acceleration=(np.array(after['dq_actual'][:7])-before['dq_actual'][:7])/scene.dt
                case['steps'].append({'tick':k+1,'before':before,'drive_before_step':drive,'after':after,
                    'acceleration':acceleration.tolist(),'acceleration_ratio':float(np.max(abs(acceleration)/scene.acceleration))})
            case['max_acceleration_ratio']=max(x['acceleration_ratio'] for x in case['steps'])
            (out/'startup_diagnostic.json').write_text(json.dumps(report,indent=2))
            print('STARTUP_CASE',name,case['max_acceleration_ratio'],flush=True)
    except Exception as error:
        import traceback;traceback.print_exc();report['error']=str(error)
        raise
    finally:
        (out/'startup_diagnostic.json').write_text(json.dumps(report,indent=2));app.close()


if __name__=='__main__':main()
