"""Local mounted synthesis. Snapshot restores exist only in this development path."""
from multipath.validation.physics import object_tracking_score
import hashlib
import json
from pathlib import Path
import numpy as np
from multipath.execution.mounted_scene import MountedScene
from multipath.validation.physics import core_pose_errors, retention_errors
from multipath.execution.transition import check_transition
from multipath.anchors.artifact import save_anchor


def predecessor_snapshot(directory, frame, provenance):
    directory=Path(directory)
    trace_path=directory/'actual_trace.json';result_path=directory/'pipeline_result.json'
    result=json.loads(result_path.read_text())
    if result['source_provenance']!=provenance:
        raise ValueError('Snapshot and local inputs have different source provenance')
    preceding=[s for s in result['segments'] if s['start']<frame]
    if frame!=0 and (not preceding or not all(s['success'] for s in preceding) or preceding[-1]['end']<frame):
        raise ValueError('Snapshot has no successful physical predecessor chain')
    state=next(s for s in json.loads(trace_path.read_text()) if abs(s['source_frame']-frame)<1e-8)
    if not state.get('physically_acquired_state') or state.get('state_origin')!='continuous_PhysX_rollout':
        raise ValueError('Reference-constructed snapshots cannot initialize development')
    state['entry_origin']='initial_state' if frame==0 else 'physically_acquired_snapshot'
    state['entry_provenance']={'trace':str(trace_path),'trace_sha256':hashlib.sha256(trace_path.read_bytes()).hexdigest(),
                             'result':str(result_path),'result_sha256':hashlib.sha256(result_path.read_bytes()).hexdigest(),
                             'source_frame':frame}
    return state


class DevelopmentScene(MountedScene):
    """Explicit development resets; never used by the continuous final executor."""
    record_mesh=True

    def capture(self,mode,frame):
        if self.record_mesh:
            super().capture(mode,frame)
        else:
            self.records.append(dict(self.state(),mode=mode,source_frame=frame,
                command_arm_q=self.qcommand[self.arm].tolist(),command_arm_dq=self.velocity_command[self.arm].tolist(),
                command_hand_q=self.qcommand[self.fingers].tolist(),command_hand_dq=self.velocity_command[self.fingers].tolist()))

    def snapshot(self):
        if self.ticks==getattr(self,'restored_tick',None):
            # Rigid-link transforms/contact buffers update on the next PhysX
            # step. The saved acquired state is authoritative at the restore.
            return dict(self.restored_state)
        s=self.state()
        s.update(command=self.qcommand.tolist(),command_velocity=self.velocity_command.tolist(),
                 entry_origin='predecessor_rollout',entry_provenance=self.lineage)
        return s

    def restore_development(self,s,align_targets=True):
        from multipath.anchors.artifact import reachable_entry
        if not reachable_entry(s) or s['scene_id']!=self.scene_id:
            raise ValueError('Development restore requires an acquired state in the same scene')
        q=self.robot.get_joint_positions().copy();dq=np.zeros_like(q)
        q[self.arm]=s['arm_q'];q[self.fingers]=s['hand_q']
        dq[self.arm]=s['arm_dq'];dq[self.fingers]=s['hand_dq']
        self.robot.set_joint_positions(q);self.robot.set_joint_velocities(dq)
        for role,obj in self.objects.items():
            state=s['objects'][role];pose=np.array(state['pose']);velocity=np.array(state['velocity'])
            obj.set_world_pose(position=pose[:3],orientation=pose[3:])
            obj.set_linear_velocity(velocity[:3]);obj.set_angular_velocity(velocity[3:])
        self.qcommand=q.copy() if align_targets else np.array(s.get('command',q)).copy()
        self.velocity_command=dq.copy() if align_targets else np.array(s.get('command_velocity',dq)).copy()
        self.robot.apply_action(self.Action(joint_positions=self.qcommand,joint_velocities=self.velocity_command))
        self.ticks=round(s['time_s']/self.dt);self.previous_dq=dq[self.arm].copy()
        self.max_velocity_ratio=0.;self.max_acceleration_ratio=0.
        self.lineage=s['entry_provenance'];self.restored_contact=s['contact']
        self.restored_tick=self.ticks;self.restored_state=dict(s,command=self.qcommand.tolist(),command_velocity=self.velocity_command.tolist())
        self.restore_count=getattr(self,'restore_count',0)+1
        # No warm-up physics step or extra source time. PhysX recomputes contact
        # on the first commanded step; solver contact caches are not serialized.

    def begin_trial(self,out,s,mesh=True):
        from multipath.mounted_xhand import ActualMeshRecorder
        self.restore_development(s);self.out=Path(out);self.out.mkdir(parents=True,exist_ok=True)
        self.records=[];self.record_mesh=mesh
        if mesh:self.recorder=ActualMeshRecorder(self.world.stage,self.out)
        # Start visual recording after the first commanded source frame; USD
        # link transforms may still be stale immediately after a state restore.


def core_index(anchor,frame,data):
    return min(max(int(round(frame))-anchor.synth_start,0),len(data['tool_pose'])-1)


def replay_padded(scene,anchor,data,controls,start,folder,method,core_only=False):
    """One uninterrupted physical execution; core-only uses the exported controls."""
    fps=scene.manifest['fps'];tpf=round(1/(fps*scene.dt))
    first,last=(anchor.core_start,anchor.core_end) if core_only else (anchor.synth_start,anchor.synth_end)
    if len(controls)!=(last-first)*tpf or abs(start['time_s']*fps-first)>1e-8:
        raise ValueError('Controls or snapshot do not match the source interval')
    scene.begin_trial(folder,start)
    entry=None;exit_state=None;held=None
    validation={'success':False,'handoff_verified':False,'synthesis_method':method,
                'context_used':'exported_core_replay' if core_only else 'physical_padded_rollout','control_dt':scene.dt,'source_fps':fps,
                'padded_rollout_complete':False,'state_resets_within_padded_rollout':0,
                'core_pose_errors':[],'post_context_retention':[],'failures':[]}
    def boundary():
        s=scene.snapshot()
        s['entry_origin']='predecessor_rollout'
        s['entry_provenance']={'snapshot':start['entry_provenance'],'padded_trace':str(scene.out/'actual_trace.json'),
                               'source_frame':scene.ticks*scene.dt*fps}
        return s
    def core_entry():
        nonlocal entry,held
        entry=boundary()
        if entry['contact']['mode']!=anchor.entry_contact:validation['failures'].append('core_entry_contact')
        if anchor.entry_contact=='grasped':held=np.array(entry['hand_object_transform'])
    try:
        if anchor.core_start==first:core_entry()
        for k,cmd in enumerate(controls):
            frame=first+(k+1)/tpf
            scene.step(cmd[:7],cmd[7:19],'EXPORTED_CORE' if core_only else 'LOCAL_PADDED',frame,cmd[19:26])
            if (k+1)%tpf:continue
            measured=scene.state()
            if frame==anchor.core_start:
                core_entry()
            if anchor.core_start<frame<=anchor.core_end:
                error=core_pose_errors(measured,data['tool_pose'][core_index(anchor,frame,data)])
                validation['core_pose_errors'].append(dict(source_frame=frame,**error))
                if not object_tracking_score(error)['feasible']:validation['failures'].append('core_object_fidelity')
                if held is not None:
                    r=retention_errors(measured,held)
                    if r['position_m']>.05 or r['rotation_rad']>.35 or not r['contact_retained']:
                        validation['failures'].append('core_grasp_retention')
            if frame==anchor.core_end:
                exit_state=boundary()
                transition=check_transition(scene,cmd[:7],anchor.exit_contact,data['tool_pose'][core_index(anchor,frame,data)])
                validation['handoff']=transition
                if not transition['success']:validation['failures'].append('core_exit_handoff')
                if anchor.exit_contact=='grasped':held=np.array(exit_state['hand_object_transform'])
                else:held=None
            if frame>anchor.core_end and held is not None:
                r=retention_errors(measured,held);validation['post_context_retention'].append(r)
                if r['position_m']>.05 or r['rotation_rad']>.35 or not r['contact_retained']:
                    validation['failures'].append('post_context_grasp_retention')
        validation['rollout_complete']=abs(scene.ticks*scene.dt*fps-last)<1e-8
        validation['padded_rollout_complete']=validation['rollout_complete'] and not core_only
    except RuntimeError as error:
        validation['failures'].append(str(error))
    finally:
        try:
            if not scene.records or scene.records[-1]['time_s']<scene.ticks*scene.dt:scene.capture('STOP',scene.ticks*scene.dt*fps)
        except Exception as error:
            validation['failures'].append(f'Final state capture failed: {error}')
        try:scene.save()
        except Exception as error:validation['failures'].append(f'Actual trace save failed: {error}')
    validation['failures']=sorted(set(validation['failures']))
    validation['success']=not validation['failures'] and validation.get('rollout_complete',False) and entry is not None and exit_state is not None
    validation['handoff_verified']=validation['success']
    validation.update(measured_velocity_ratio=scene.max_velocity_ratio,measured_acceleration_ratio=scene.max_acceleration_ratio,
        measured_acceleration_policy='diagnostic_relative_to_curobo_nominal_budget',
        nominal_acceleration_budget=np.asarray(scene.acceleration).tolist(),
        configured_drive_effort_caps=np.asarray(scene.robot._articulation_view.get_max_efforts()).reshape(-1)[np.r_[scene.arm,scene.fingers]].tolist(),
        effort_measurement_note='Measured joint efforts include reaction loads; not an isolated drive torque measurement',
        collision_validation_note='PhysX contact response enabled; exhaustive penetration and self-collision acceptance not implemented')
    np.savez_compressed(scene.out/('core_controls.npz' if core_only else 'padded_controls.npz'),commands=controls)
    if entry is not None:
        lo=(anchor.core_start-first)*tpf;hi=(anchor.core_end-first)*tpf
        save_anchor(scene.out/'anchors',anchor,entry,exit_state or scene.state(),controls[lo:hi],validation,scene.manifest['provenance'])
    (scene.out/'local_result.json').write_text(json.dumps(validation,indent=2))
    (scene.out/'pipeline_result.json').write_text(json.dumps(dict(validation,source='oracle',source_fps=fps,
        source_start_s=start['time_s'],actual_duration_s=scene.ticks*scene.dt-start['time_s'],
        source_duration_s=(last-first)/fps,full_pipeline_success=False,
        table_z=float(scene.table['pose'][2]+.02)),indent=2))
    return validation


def validate_exported_core(scene,anchor,data,padded_folder,folder,artifact_root=None):
    """Gate 2 of local development: replay exactly the exported core from its measured entry."""
    padded_folder=Path(padded_folder);folder=Path(folder)
    padded=json.loads((padded_folder/'local_result.json').read_text())
    if not padded['success'] or not padded['padded_rollout_complete']:
        raise ValueError('Exported core replay requires a successful padded rollout')
    original=padded_folder/'anchors'/anchor.id
    entry=json.loads((original/'entry.json').read_text())
    controls=np.load(original/'commands.npz',allow_pickle=False)['commands']
    replay=replay_padded(scene,anchor,data,controls,entry,folder,'exported_core_replay',core_only=True)
    validation=dict(padded,success=bool(replay['success']),handoff_verified=bool(replay['handoff_verified']),
                    core_replay_success=bool(replay['success']),state_resets_within_core_replay=0,
                    failures=replay['failures'],
                    padded_evidence={'result':str(padded_folder/'local_result.json'),
                                     'trace':str(padded_folder/'actual_trace.json')},
                    core_replay=replay,core_replay_evidence={'result':str(folder/'local_result.json'),
                                                          'trace':str(folder/'actual_trace.json')})
    # Publish only after both executions pass. The core replay's measured exit is
    # the canonical exit used for subsequent bridge planning.
    core_artifact=folder/'anchors'/anchor.id
    exit_state=json.loads((core_artifact/'exit.json').read_text())
    save_anchor(folder/'anchors',anchor,entry,exit_state,controls,validation,scene.manifest['provenance'])
    if validation['success'] and artifact_root is not None:
        # Other Anchor workers may start a connection as soon as this appears.
        # Publish the complete directory in one rename, never half an artifact.
        import tempfile
        artifact_root=Path(artifact_root);artifact_root.mkdir(parents=True,exist_ok=True)
        with tempfile.TemporaryDirectory(dir=artifact_root,prefix='.publishing_') as temporary:
            save_anchor(temporary,anchor,entry,exit_state,controls,validation,scene.manifest['provenance'])
            (Path(temporary)/anchor.id).rename(artifact_root/anchor.id)
    (folder/'local_gate_result.json').write_text(json.dumps(validation,indent=2))
    return validation



# Keep the public development entry point; the optimizer has its own module.
from multipath.anchors.mpc import mounted_mpc
