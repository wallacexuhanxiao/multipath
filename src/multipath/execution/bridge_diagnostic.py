"""Small physical attribution: source-time transport and controller takeover.

Every trial begins from the same real acquired snapshot. Retention threshold
failures remain recorded failures even when diagnostic execution continues.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from multipath.validation.physics import retention_errors
from multipath.execution.wrench_handoff import takeover_bias


def run(scene,start,commands,planner,goal,target,out):
    out=Path(out);results=[];fps=scene.manifest['fps'];begin=start['time_s']*fps
    for kind in ['planned','wrench_continuous','previous_drive_continuation']:
        trial=out/kind;scene.begin_trial(trial,start,mesh=True);scene.capture('INITIAL',begin)
        resets=scene.restore_count;bias=takeover_bias(start,commands[0],scene.cfg['controller'])
        rows=[];failed=False;error=None
        for k,command in enumerate(commands):
            c=command.copy();phase=k/max(1,len(commands)-1)
            scene.wrench_bias=bias*(1-3*phase**2+2*phase**3) if kind=='wrench_continuous' else np.zeros(6)
            if kind=='previous_drive_continuation':
                c[:7]=start['command_wrist'];c[19:25]=start['command_twist']
            frame=begin+(k+1)*scene.dt*fps
            try:
                scene.step_control(c,'RELAXED_DIAGNOSTIC',frame);s=scene.state();r=retention_errors(s,start['hand_object_transform'])
                violation=r['position_m']>.05 or r['rotation_rad']>.35 or not r['contact_retained']
                failed|=violation;clearance=planner.measured_clearance()
                collisions=planner.measured_mesh_collisions() if min(clearance.values())<0 else []
                rows.append(dict(source_frame=frame,retention=r,retention_violation=violation,
                    proxy_clearance_m=clearance,mesh_collisions=collisions,state=s,wrench=scene.last_wrench.tolist()))
                if collisions or not r['contact_retained']:
                    error='actual_mesh_collision' if collisions else 'opposition_contact_lost';break
            except Exception as e:error=str(e);break
        scene.wrench_bias=np.zeros(6);scene.save()
        transition=scene.check_transition(np.asarray(goal['wrist_xyz_wxyz']),'grasped',expected_object=target)
        info=dict(kind=kind,source_duration_budget_s=len(commands)*scene.dt,
            actual_duration_s=(scene.ticks*scene.dt-start['time_s']),completed=len(rows)==len(commands),
            retention_failed=failed,error=error,transition=transition,
            bridge_success=bool(kind!='previous_drive_continuation' and not failed and error is None and len(rows)==len(commands) and transition['success']),
            resets_within_rollout=scene.restore_count-resets,initial_wrench_bias=bias.tolist() if kind=='wrench_continuous' else [0.]*6,
            max_relative_rotation_rad=max((r['retention']['rotation_rad'] for r in rows),default=0.),
            max_relative_position_m=max((r['retention']['position_m'] for r in rows),default=0.))
        (trial/'physics_steps.json').write_text(json.dumps(rows,indent=2))
        (trial/'end_state.json').write_text(json.dumps(scene.snapshot(),indent=2))
        (trial/'diagnostic_result.json').write_text(json.dumps(info,indent=2));results.append(info)
        print('BRIDGE_DIAGNOSTIC',json.dumps(info),flush=True)
    return results
