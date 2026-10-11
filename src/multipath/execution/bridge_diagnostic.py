"""Small physical attribution: source-time transport and controller takeover.

Every trial begins from the same real acquired snapshot. Retention threshold
failures remain recorded failures even when diagnostic execution continues.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from multipath.validation.physics import retention_errors, OBJECT_POSITION_TOLERANCE_M, OBJECT_ROTATION_TOLERANCE_RAD
from multipath.execution.wrench_handoff import takeover_bias,balanced_force_profile,carried_gravity_wrench


def run(scene,start,commands,planner,goal,target,out,components=False,balanced=False,loaded=False,dynamics=False,bandwidth=False):
    out=Path(out);results=[];fps=scene.manifest['fps'];begin=start['time_s']*fps
    kinds=['rigid_dynamics_bandwidth'] if dynamics and bandwidth else ['rigid_dynamics_balanced'] if dynamics and balanced else ['rigid_dynamics','rigid_dynamics_continuous'] if dynamics else ['load_compensated_cubic','load_compensated_balanced'] if loaded else ['balanced_force_torque_continuous'] if balanced else ['torque_continuous','force_continuous'] if components else ['planned','wrench_continuous','previous_drive_continuation']
    for kind in kinds:
        trial=out/kind;scene.begin_trial(trial,start,mesh=True);scene.capture('INITIAL',begin)
        resets=scene.restore_count;scene.transport_load=loaded
        load=carried_gravity_wrench(scene) if loaded else None
        feedforward=None
        if dynamics:
            from multipath.execution.transport_dynamics import TransportDynamics
            model=TransportDynamics(scene,planner.carry);feedforward=model.commands(scene,start,commands)
            load=feedforward[0]
        bias=takeover_bias(start,commands[0],scene.cfg['controller'],new_feedforward=load)
        mass=float(np.sum(scene.av.get_masses())+scene.objects['tool'].get_mass())
        use_balanced=balanced or kind=='load_compensated_balanced'
        if bandwidth:
            from multipath.execution.wrench_handoff import bandwidth_handoff_profile
            rotation=Rotation.from_quat(np.asarray(start['wrist_xyz_wxyz'])[[4,5,6,3]]).as_matrix()
            handoff_profile=bandwidth_handoff_profile(len(commands),scene.dt,model.mass,rotation@model.inertia@rotation.T,scene.cfg['controller'])
        force_profile=balanced_force_profile(len(commands),scene.dt,mass,scene.cfg['controller']['wrist_kp'],scene.cfg['controller']['wrist_kd']) if use_balanced else None
        rows=[];failed=False;error=None
        for k,command in enumerate(commands):
            c=command.copy();phase=k/max(1,len(commands)-1)
            scene.wrench_bias=bias*(1-3*phase**2+2*phase**3) if loaded or kind in ['wrench_continuous','torque_continuous','force_continuous'] else np.zeros(6)
            if dynamics:
                scene.wrench_bias=feedforward[k].copy()
                if kind=='rigid_dynamics_continuous':scene.wrench_bias+=bias*(1-3*phase**2+2*phase**3)
                if bandwidth:scene.wrench_bias+=bias*handoff_profile[k]
            if kind=='torque_continuous':scene.wrench_bias[:3]=0.
            if kind=='force_continuous':scene.wrench_bias[3:]=0.
            if use_balanced:
                scene.wrench_bias=np.r_[bias[:3]*force_profile[k],bias[3:]*(1-3*phase**2+2*phase**3)]
                if dynamics:scene.wrench_bias+=feedforward[k]
            if kind=='previous_drive_continuation':
                c[:7]=start['command_wrist'];c[19:25]=start['command_twist']
            frame=begin+(k+1)*scene.dt*fps
            try:
                scene.step_control(c,'RELAXED_DIAGNOSTIC',frame);s=scene.state();r=retention_errors(s,start['hand_object_transform'])
                violation=r['position_m']>OBJECT_POSITION_TOLERANCE_M or r['rotation_rad']>OBJECT_ROTATION_TOLERANCE_RAD or not r['contact_retained']
                failed|=violation;clearance=planner.measured_clearance()
                collisions=planner.measured_mesh_collisions() if min(clearance.values())<0 else []
                rows.append(dict(source_frame=frame,retention=r,retention_violation=violation,
                    proxy_clearance_m=clearance,mesh_collisions=collisions,state=s,wrench=scene.last_wrench.tolist()))
                if collisions or not r['contact_retained']:
                    error='actual_mesh_collision' if collisions else 'opposition_contact_lost';break
            except Exception as e:error=str(e);break
        end_state=scene.snapshot();scene.wrench_bias=np.zeros(6);scene.save()
        transition=scene.check_transition(np.asarray(goal['wrist_xyz_wxyz']),'grasped',expected_object=target)
        info=dict(kind=kind,source_duration_budget_s=len(commands)*scene.dt,
            actual_duration_s=(scene.ticks*scene.dt-start['time_s']),completed=len(rows)==len(commands),
            retention_failed=failed,error=error,transition=transition,
            bridge_success=bool(kind!='previous_drive_continuation' and not failed and error is None and len(rows)==len(commands) and transition['success']),
            resets_within_rollout=scene.restore_count-resets,initial_wrench_bias=(np.r_[np.zeros(3),bias[3:]] if kind=='torque_continuous' else np.r_[bias[:3],np.zeros(3)] if kind=='force_continuous' else bias if kind=='wrench_continuous' else np.zeros(6)).tolist(),
            max_relative_rotation_rad=max((r['retention']['rotation_rad'] for r in rows),default=0.),
            max_relative_position_m=max((r['retention']['position_m'] for r in rows),default=0.))
        (trial/'physics_steps.json').write_text(json.dumps(rows,indent=2))
        if use_balanced:info.update(effective_mass_kg=mass,force_profile=force_profile.tolist(),initial_wrench_bias=bias.tolist())
        if loaded:info.update(carried_gravity_wrench_at_entry=load.tolist(),initial_wrench_bias=bias.tolist())
        if dynamics:info.update(transport_dynamics=model.report(),initial_wrench_bias=bias.tolist(),feedforward_wrenches=feedforward.tolist())
        if bandwidth:info['handoff_profile']=handoff_profile.tolist()
        (trial/'end_state.json').write_text(json.dumps(end_state,indent=2))
        (trial/'diagnostic_result.json').write_text(json.dumps(info,indent=2));results.append(info)
        print('BRIDGE_DIAGNOSTIC',json.dumps(info),flush=True)
    return results
