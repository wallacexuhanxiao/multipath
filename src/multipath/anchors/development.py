"""Local mounted synthesis. Snapshot restores exist only in this development path."""
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
    if not preceding or not all(s['success'] for s in preceding) or preceding[-1]['end']<frame:
        raise ValueError('Snapshot has no successful physical predecessor chain')
    state=next(s for s in json.loads(trace_path.read_text()) if abs(s['source_frame']-frame)<1e-8)
    if not state.get('physically_acquired_state') or state.get('state_origin')!='continuous_PhysX_rollout':
        raise ValueError('Reference-constructed snapshots cannot initialize development')
    state['entry_origin']='physically_acquired_snapshot'
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
            self.records.append(dict(self.state(),mode=mode,source_frame=frame))

    def snapshot(self):
        if self.ticks==getattr(self,'restored_tick',None):
            # Rigid-link transforms/contact buffers update on the next PhysX
            # step. The saved acquired state is authoritative at the restore.
            return dict(self.restored_state)
        s=self.state()
        s.update(command=self.qcommand.tolist(),command_velocity=self.velocity_command.tolist(),
                 entry_origin='predecessor_rollout',entry_provenance=self.lineage)
        return s

    def restore_development(self,s):
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
        self.qcommand=np.array(s.get('command',q)).copy()
        self.velocity_command=np.array(s.get('command_velocity',dq)).copy()
        self.robot.apply_action(self.Action(joint_positions=self.qcommand,joint_velocities=self.velocity_command))
        self.ticks=round(s['time_s']/self.dt);self.previous_dq=dq[self.arm].copy()
        self.max_velocity_ratio=0.;self.max_acceleration_ratio=0.
        self.lineage=s['entry_provenance'];self.restored_contact=s['contact']
        self.restored_tick=self.ticks;self.restored_state=dict(s)
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


def replay_padded(scene,anchor,data,controls,start,folder,method):
    """One uninterrupted padded execution; save only CORE controls as an Anchor."""
    fps=scene.manifest['fps'];tpf=round(1/(fps*scene.dt))
    if len(controls)!=(anchor.synth_end-anchor.synth_start)*tpf or abs(start['time_s']*fps-anchor.synth_start)>1e-8:
        raise ValueError('Padded controls or snapshot do not match the source interval')
    scene.begin_trial(folder,start)
    entry=None;exit_state=None;held=None
    validation={'success':False,'handoff_verified':False,'synthesis_method':method,
                'context_used':'physical_padded_rollout','control_dt':scene.dt,'source_fps':fps,
                'padded_rollout_complete':False,'state_resets_within_padded_rollout':0,
                'core_pose_errors':[],'post_context_retention':[],'failures':[]}
    def boundary():
        s=scene.snapshot()
        s['entry_origin']='predecessor_rollout'
        s['entry_provenance']={'snapshot':start['entry_provenance'],'padded_trace':str(scene.out/'actual_trace.json'),
                               'source_frame':scene.ticks*scene.dt*fps}
        return s
    try:
        if anchor.core_start==anchor.synth_start:entry=boundary()
        for k,cmd in enumerate(controls):
            frame=anchor.synth_start+(k+1)/tpf
            scene.step(cmd[:7],cmd[7:19],'LOCAL_PADDED',frame,cmd[19:26])
            if (k+1)%tpf:continue
            measured=scene.state()
            if frame==anchor.core_start:
                entry=boundary()
                if entry['contact']['mode']!=anchor.entry_contact:validation['failures'].append('core_entry_contact')
                if anchor.entry_contact=='grasped':held=np.array(entry['hand_object_transform'])
            if anchor.core_start<frame<=anchor.core_end:
                error=core_pose_errors(measured,data['tool_pose'][core_index(anchor,frame,data)])
                validation['core_pose_errors'].append(dict(source_frame=frame,**error))
                if error['position_m']>.05 or error['rotation_rad']>.35:validation['failures'].append('core_object_fidelity')
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
        validation['padded_rollout_complete']=abs(scene.ticks*scene.dt*fps-anchor.synth_end)<1e-8
    except RuntimeError as error:
        validation['failures'].append(str(error))
    finally:
        if not scene.records or scene.records[-1]['time_s']<scene.ticks*scene.dt:scene.capture('STOP',scene.ticks*scene.dt*fps)
        scene.save()
    validation['failures']=sorted(set(validation['failures']))
    validation['success']=not validation['failures'] and validation['padded_rollout_complete'] and entry is not None and exit_state is not None
    validation['handoff_verified']=validation['success']
    validation.update(measured_velocity_ratio=scene.max_velocity_ratio,measured_acceleration_ratio=scene.max_acceleration_ratio)
    np.savez_compressed(scene.out/'padded_controls.npz',commands=controls)
    if entry is not None:
        lo=(anchor.core_start-anchor.synth_start)*tpf;hi=(anchor.core_end-anchor.synth_start)*tpf
        save_anchor(scene.out/'anchors',anchor,entry,exit_state or scene.state(),controls[lo:hi],validation,scene.manifest['provenance'])
    (scene.out/'local_result.json').write_text(json.dumps(validation,indent=2))
    (scene.out/'pipeline_result.json').write_text(json.dumps(dict(validation,source='oracle',source_fps=fps,
        source_start_s=start['time_s'],actual_duration_s=scene.ticks*scene.dt-start['time_s'],
        source_duration_s=(anchor.synth_end-anchor.synth_start)/fps,full_pipeline_success=False,
        table_z=float(scene.table['pose'][2]+.02)),indent=2))
    return validation


def mounted_mpc(scene,anchor,data,baseline,start,out,seed=0):
    """Small shooting MPC: arm + finger residuals evaluated by mounted PhysX.

    Candidate resets are explicit development-only predictions. A selected plan
    is subsequently replayed from the acquired snapshot without internal resets.
    """
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    scene.begin_trial(out/'selected_prefix',start,mesh=False)
    fps=scene.manifest['fps'];tpf=round(1/(fps*scene.dt))
    limits=np.asarray(scene.robot._articulation_view.get_dof_limits()).reshape(-1,2)
    lower=limits[np.r_[scene.arm,scene.fingers],0];upper=limits[np.r_[scene.arm,scene.fingers],1]
    rng=np.random.default_rng(seed);chosen=baseline.copy();history=[];trial_id=0
    for begin in range(0,len(baseline),6*tpf):
        stop=min(begin+12*tpf,len(baseline));commit=min(6*tpf,len(baseline)-begin)
        state=scene.snapshot();base=baseline[begin:stop].copy();n=len(base)
        duration=n*scene.dt
        scale=np.r_[np.minimum(scene.velocity*duration*.1,(upper-lower)[:7]*.02),(upper-lower)[7:]*.04]
        mean=np.zeros(19);sigma=scale.copy();best=None;best_score=float('inf')
        # Carry the previous MPC position/velocity residual into this horizon;
        # tracking error must not be reinterpreted as feedforward (r9 failure).
        phase=np.arange(1,n+1)/n;blend=np.sin(np.pi*phase)**2
        blend_velocity=np.pi*np.sin(2*np.pi*phase)/duration
        qprevious=np.array(state['command'])[np.r_[scene.arm,scene.fingers]]
        vprevious=np.array(state['command_velocity'])[scene.arm]
        if begin:
            offset=qprevious-baseline[begin-1,:19]
            speed=np.r_[vprevious-baseline[begin-1,19:26],np.zeros(12)]
            base[:,:19]+=(2*phase**3-3*phase**2+1)[:,None]*offset+(phase**3-2*phase**2+phase)[:,None]*duration*speed
            base[:,19:26]+=(6*phase**2-6*phase)[:,None]/duration*offset[:7]+(3*phase**2-4*phase+1)[:,None]*speed[:7]
        else:
            qprevious=np.r_[state['arm_q'],state['hand_q']];vprevious=np.array(state['arm_dq'])
        for iteration in range(2):
            candidates=[mean]+[rng.normal(mean,sigma) for _ in range(5)]
            scored=[]
            for residual in candidates:
                trial_id+=1;control=base.copy();control[:,:19]+=blend[:,None]*residual
                control[:,19:26]=base[:,19:26]+blend_velocity[:,None]*residual[:7]
                velocity=np.diff(np.vstack((qprevious[:7],control[:,:7])),axis=0)/scene.dt
                acceleration=np.diff(np.vstack((vprevious,velocity)),axis=0)/scene.dt
                feasible=(np.all(control[:,:19]>=lower) and np.all(control[:,:19]<=upper)
                          and np.max(np.abs(velocity)/scene.velocity)<=1.05
                          and np.max(np.abs(acceleration)/scene.acceleration)<=1.05)
                trial={'trial':trial_id,'begin_tick':begin,'feasible_controls':bool(feasible),'score':None}
                if not feasible:history.append(trial);continue
                folder=out/f'prediction_{trial_id:04d}';scene.begin_trial(folder,state,mesh=False)
                score=0.;samples=0;failure=None;acquired=None
                try:
                    for j,cmd in enumerate(control):
                        frame=anchor.synth_start+(begin+j+1)/tpf
                        scene.step(cmd[:7],cmd[7:19],'MPC_PREDICTION',frame,cmd[19:26])
                        if (j+1)%tpf:continue
                        measured=scene.state()
                        if anchor.core_start<frame<=anchor.core_end:
                            e=core_pose_errors(measured,data['tool_pose'][core_index(anchor,frame,data)])
                            score+=(e['position_m']/.05)**2+(e['rotation_rad']/.35)**2;samples+=1
                            if measured['contact']['mode']=='grasped' and acquired is None:
                                acquired=np.array(measured['hand_object_transform'])
                            if acquired is not None:
                                r=retention_errors(measured,acquired)
                                score+=(r['position_m']/.05)**2+(r['rotation_rad']/.35)**2+2*(not r['contact_retained'])
                        if frame>=anchor.core_end:
                            score+=4*(measured['contact']['mode']!=anchor.exit_contact);samples+=1
                    score=score/max(1,samples)+.01*float(np.mean((residual/np.maximum(scale,1e-9))**2))
                except RuntimeError as error:
                    failure=str(error);score=float('inf')
                (folder/'actual_trace.json').write_text(json.dumps(scene.records))
                trial.update(score=score if np.isfinite(score) else None,error=failure);history.append(trial)
                scored.append((score,residual))
                if score<best_score:best_score=score;best=control.copy()
            if scored:
                elite=[x[1] for x in sorted(scored,key=lambda x:x[0])[:2] if np.isfinite(x[0])]
                if elite:mean=np.mean(elite,axis=0);sigma=np.maximum(np.std(elite,axis=0),scale*.25)
        (out/'search.json').write_text(json.dumps(history,indent=2))
        if best is None:raise RuntimeError(f'Mounted MPC found no physically feasible prediction at tick {begin}')
        scene.begin_trial(out/f'committed_{begin:04d}',state,mesh=False)
        for j,cmd in enumerate(best[:commit]):
            scene.step(cmd[:7],cmd[7:19],'MPC_COMMIT',anchor.synth_start+(begin+j+1)/tpf,cmd[19:26])
        (scene.out/'actual_trace.json').write_text(json.dumps(scene.records))
        chosen[begin:begin+commit]=best[:commit]
        print('MOUNTED_MPC',begin,'score',best_score,flush=True)
    return chosen
