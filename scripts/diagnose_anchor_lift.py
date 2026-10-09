"""Compare an observed failed interaction with a vertical lift from its grip.

Both branches reacquire the real initial/predecessor state. Only commands are
changed. This attribution probe is not an Anchor or end-to-end acceptance.
"""
import argparse,json
from pathlib import Path


def main():
    p=argparse.ArgumentParser()
    for key in ('schedule','proposals','entry-run','local-evaluation','xhand-urdf',
                'object-cache','floating-config','output-dir'):
        p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--anchor-id',required=True)
    args=p.parse_args();out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    from multipath.execution.startup import watch_dependency_imports,start_simulation
    watch_dependency_imports(out)
    import numpy as np
    from scipy.interpolate import BPoly
    from scipy.spatial.transform import Rotation
    from multipath.oracle_schedule import OracleSchedule
    from multipath.execution.floating_scene import FloatingScene
    from multipath.execution.wrench_handoff import segment_wrenches
    from multipath.planning.timing import physics_ticks_per_frame
    schedule=OracleSchedule.load(args.schedule);anchor=schedule.anchors[args.anchor_id]
    manifest=json.loads((args.proposals/'manifest.json').read_text())
    predecessor=json.loads((args.entry_run/'pipeline_result.json').read_text())
    if predecessor['source_provenance']!=manifest['provenance']:
        raise ValueError('Physical predecessor has different source provenance')
    trace=json.loads((args.local_evaluation/'actual_trace.json').read_text())
    observed=next(s for s in trace if s['source_frame']>=(anchor.core_start+anchor.core_end)/2
                  and s['contact']['mode']=='grasped')
    begin=round(observed['source_frame']);ticks=physics_ticks_per_frame(schedule.fps,FloatingScene.dt)
    original=np.load(args.local_evaluation/'commands.npz')['commands']
    count=(anchor.core_end-anchor.core_start)*ticks
    if len(original)<count:raise ValueError('Observed commands do not reach the core endpoint')
    data=np.load(args.proposals/manifest['anchors'][anchor.id])
    lift=float(data['tool_pose'][anchor.core_end-anchor.synth_start,2,3]-
               data['tool_pose'][begin-anchor.synth_start,2,3])
    if lift<=0:raise ValueError('The remaining source interaction does not request upward motion')
    app=None
    try:
        app=start_simulation(out)
        initial=predecessor['initial_state']
        scene=FloatingScene(args,manifest,out/'initial',initial['command_wrist'],initial_hand=initial['command_hand_q'])
        from pxr import UsdPhysics,UsdShade
        materials=[]
        for prim in scene.world.stage.Traverse():
            if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
            mat,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(materialPurpose='physics')
            item={'collider':str(prim.GetPath()),'material':str(mat.GetPath()) if mat else None}
            if mat:
                api=UsdPhysics.MaterialAPI(mat.GetPrim())
                item.update(static_friction=api.GetStaticFrictionAttr().Get(),
                            dynamic_friction=api.GetDynamicFrictionAttr().Get())
            materials.append(item)

        def forces():
            from multipath.execution.transport_dynamics import hand_gravity_torque
            normal=np.asarray(scene.contacts.get_contact_force_matrix(dt=scene.dt))
            friction,_,counts,starts=scene.contacts.get_friction_data(dt=scene.dt)
            friction=np.asarray(friction);counts=np.asarray(counts);starts=np.asarray(starts)
            aggregate=np.zeros((*counts.shape,3))
            for idx in np.ndindex(counts.shape):
                k=int(starts[idx]);n=int(counts[idx]);aggregate[idx]=friction[k:k+n].sum(0)
            return dict(normal_force_on_hand_n=normal.reshape(-1,3).sum(0).tolist(),
                friction_force_on_hand_n=aggregate.reshape(-1,3).sum(0).tolist(),
                hand_gravity_compensation_moment_n_m=hand_gravity_torque(scene,scene.root_motion()[0]).tolist(),
                friction_anchor_count=int(counts.sum()),
                total_hand_force_on_tool_n=-(normal.reshape(-1,3).sum(0)+aggregate.reshape(-1,3).sum(0)))

        reports=[]
        for kind in ('observed_motion','vertical_same_grip'):
            folder=out/kind;scene.begin_initial_trial(folder,mesh=False)
            for window in schedule.windows:
                if window['start']>=anchor.core_start:break
                if window['mode']!='RELAXED':raise ValueError('This acquisition probe requires a free-motion predecessor')
                command=np.load(args.entry_run/f'relaxed_{window["start"]}_{window["end"]}.npz')['commands']
                segment=next(s for s in predecessor['segments'] if s['start']==window['start'])
                free=segment.get('planning',segment).get('free_motion_dynamics_feedforward',False)
                bias=segment_wrenches(scene,scene.snapshot(),command,free_motion=free)
                for k,c in enumerate(command):
                    scene.wrench_bias=bias[k];scene.step_control(c,'PHYSICAL_PREDECESSOR',window['start']+(k+1)/ticks)
            scene.begin_recording(folder,mesh=True)
            start=scene.snapshot();bias=segment_wrenches(scene,start,original)
            split=(begin-anchor.core_start)*ticks
            for k,c in enumerate(original[:split]):
                scene.wrench_bias=bias[k];scene.step_control(c,'OBSERVED_ACQUISITION',anchor.core_start+(k+1)/ticks)
            reached=scene.snapshot();commands=original[split:count].copy()
            if kind=='vertical_same_grip':
                last=np.r_[scene.command_pose,scene.hand_command,scene.command_twist]
                duration=len(commands)*scene.dt;time=np.arange(1,len(commands)+1)*scene.dt
                curve=BPoly.from_derivatives([0.,duration],[[last[:3],last[19:22],np.zeros(3)],
                    [last[:3]+np.array([0.,0.,lift]),np.zeros(3),np.zeros(3)]])
                angular=BPoly.from_derivatives([0.,duration],[[np.zeros(3),last[22:25],np.zeros(3)],
                    [np.zeros(3),np.zeros(3),np.zeros(3)]])
                commands[:,:3]=curve(time);commands[:,19:22]=curve.derivative()(time)
                commands[:,3:7]=(Rotation.from_rotvec(angular(time))*
                    Rotation.from_quat(last[[4,5,6,3]])).as_quat()[:,[3,0,1,2]]
                commands[:,22:25]=angular.derivative()(time);commands[:,7:19]=last[7:19]
                bias=segment_wrenches(scene,reached,commands);offset=0
            else:offset=split
            samples=[]
            for k,c in enumerate(commands):
                frame=begin+(k+1)/ticks
                scene.wrench_bias=bias[k+offset];scene.step_control(c,kind.upper(),frame)
                if (k+1)%ticks==0:
                    f=forces();f['total_hand_force_on_tool_n']=f['total_hand_force_on_tool_n'].tolist()
                    samples.append(dict(source_frame=frame,**f))
            scene.save();end=scene.snapshot()
            report=dict(branch=kind,start_frame=begin,end_frame=anchor.core_end,
                source_time_budget_s=(anchor.core_end-begin)/schedule.fps,
                reached_start_contact=reached['contact'],source_lift_command_m=lift,
                actual_tool_lift_m=end['objects']['tool']['pose'][2]-reached['objects']['tool']['pose'][2],
                actual_wrist_lift_m=end['wrist_xyz_wxyz'][2]-reached['wrist_xyz_wxyz'][2],
                end_contact=end['contact'],forces=samples,tool_weight_n=scene.physical_properties['tool_mass_kg']*9.81,
                physical_acceptance=False,state_assignment_between_segments=False)
            (folder/'lift_probe.json').write_text(json.dumps(report,indent=2))
            reports.append(report);print('LIFT_ATTRIBUTION',kind,report['actual_tool_lift_m'],report['actual_wrist_lift_m'],flush=True)
        (out/'lift_attribution.json').write_text(json.dumps(dict(materials=materials,branches=reports,
            source_provenance=manifest['provenance'],validation_scope='diagnostic only; no task acceptance'),indent=2))
    finally:
        if app is not None:app.close()


if __name__=='__main__':main()
