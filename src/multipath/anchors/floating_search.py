"""Replay -> Cartesian MPPI -> small residual PPO for floating XHand.

Same source clock, core objective and contact evidence as the mounted path;
no arm IK, arm dynamic fitting, or full robot reference dependency.
"""
import json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from multipath.anchors.mpc import WindowEvaluation,mppi_mean,RefinementRequired
from multipath.anchors.floating import controls_from_poses,validate_commands,object_relative_command,object_relative_handoff,hand_command_boundary,project_hand_commands,grasp_command_bounds,stopping_context,acquisition_preload,AcquiredGraspProposal,grasp_drive_transform
from multipath.anchors.ppo import MountedResidualEnv,LocalPPOSolver,PPOConfig
from multipath.planning.timing import physics_ticks_per_frame
from multipath.validation.physics import object_tracking_parameters,object_optimization_parameters


def apply_residual(scene,baseline,start,residual,tpf,envelope,anchor=None):
    """Interpolate world-frame wrist residuals and enforce hand joint/speed bounds."""
    ticks=len(baseline);sample=np.arange(ticks+1)/tpf
    delta=np.vstack((np.zeros(18),residual));grid=np.arange(len(delta))
    interp=np.stack([np.interp(sample,grid,delta[:,j]) for j in range(18)],axis=1)
    interp*=np.asarray(envelope)[:,None]
    poses=np.vstack((start['wrist_xyz_wxyz'],baseline[:,:7]));hands=np.vstack((hand_command_boundary(start),baseline[:,7:19]))
    poses[:,:3]+=interp[:,:3]
    rotations=Rotation.from_rotvec(interp[:,3:6])*Rotation.from_quat(poses[:,[4,5,6,3]])
    poses[:,3:]=rotations.as_quat()[:,[3,0,1,2]];hands+=interp[:,6:]
    low,high=grasp_command_bounds(scene,start,anchor)
    hands=np.clip(hands,low,high)
    for k in range(1,len(hands)):hands[k]=np.clip(hands[k],hands[k-1]-scene.hand_velocity*scene.dt,hands[k-1]+scene.hand_velocity*scene.dt)
    controls=controls_from_poses(poses,hands,scene.dt)
    # Appendix reproduction samples pose/finger residuals; it does not add
    # finite-differenced random pose noise to the wrist velocity feedforward.
    # Keep the nominal source/bridge command twist and let physical feedback
    # execute the sampled pose target. RELAXED planning is unaffected.
    controls[:,19:25]=baseline[:,19:25]
    return project_hand_commands(scene,controls,start,anchor)


class FloatingResidualEnv(MountedResidualEnv):
    def __init__(self,scene,anchor,data,baseline,start,begin,out,*,entry_acquirer=None,controller_handoff=False,acquisition_reference=None):
        self.scene,self.anchor,self.data=scene,anchor,data
        self.entry_acquirer=entry_acquirer;self.controller_handoff=controller_handoff
        self.baseline,self.start,self.begin=project_hand_commands(scene,baseline,start,anchor),start,begin
        self.acquisition_reference=(self.baseline.copy() if acquisition_reference is None else
            project_hand_commands(scene,acquisition_reference,start,anchor))
        if self.acquisition_reference.shape!=self.baseline.shape:
            raise ValueError('Acquisition nominal reference must match the local source interval')
        self.out=Path(out);self.out.mkdir(parents=True,exist_ok=True)
        self.fps=scene.manifest['fps'];self.tpf=physics_ticks_per_frame(self.fps,scene.dt);self.horizon=len(baseline)//self.tpf
        if self.horizon<1 or len(baseline)%self.tpf or not validate_commands(scene,self.baseline,start)['feasible']:
            raise ValueError('PPO requires feasible whole-source-frame local commands')
        if not np.isclose(start['time_s']*self.fps,anchor.synth_start+begin/self.tpf,atol=1e-8,rtol=0):
            raise ValueError('PPO entry does not match local source time')
        self.prior=self.baseline[self.tpf-1::self.tpf,:19].copy();self.hand_velocity=scene.hand_velocity;self.trial=0
        self.metadata={'anchor':anchor.id,'begin_tick':begin,'source_start_frame':anchor.synth_start+begin/self.tpf,
            'source_frames':self.horizon,'source_fps':self.fps,'physics_dt':scene.dt,'entry_origin':start.get('entry_origin'),
            'entry_provenance':start.get('entry_provenance'),'source_provenance':scene.manifest['provenance'],
            'action':'wrist_xyz_rotvec_and_12_fingers','action_scales':self.action_scales.tolist(),'residual_center':'local_floating_replay',
            'training_scene':'floating_PhysX','residual_velocity_feedforward':'nominal_replay_unchanged','objective':'Appendix C.3+C.7+C.8',
            'early_termination':'C.4 weighted error > C; no rollout past tracking failure',
            'evaluation_reward':'C.9 object tracking only',
            'optimization_parameters':object_optimization_parameters(),
            'residual_after_grasp':'wrist pose residual retained over acquired-grasp nominal command',
            'acquisition_residual_velocity':'nominal drive history; pose residuals are not differentiated',
            'tracking_parameters':object_tracking_parameters(),
            'finger_projection':'URDF_limits_slew_and_native_contact_preload',
            'clock_stretched':False,'state_resets_within_episode':0,'domain_randomization':False,'arm_ik_used':False}

    def observation(self,state):
        i=min(self.k,self.horizon-1);idx=min(round(self.begin/self.tpf+i+1),len(self.data['tool_pose'])-1);ref=self.data['tool_pose'][idx]
        expected=np.r_[ref[:3,3],Rotation.from_matrix(ref[:3,:3]).as_quat()[[3,0,1,2]]]
        obs=np.r_[state['wrist_xyz_wxyz'],state['wrist_twist'],state['hand_q'],np.asarray(state['hand_dq'])/self.hand_velocity,
            state['objects']['tool']['pose'],state['objects']['tool']['velocity'],state['objects']['target']['pose'],
            np.asarray(state['hand_object_transform'])[:3].reshape(-1),self.prior[i],expected,self.k/self.horizon,self.previous_action]
        if not np.isfinite(obs).all():raise RuntimeError('Non-finite floating PPO observation')
        return obs.astype(np.float32)

    def reset(self):
        self.trial+=1
        if self.entry_acquirer is None:self.scene.begin_trial(self.out/'episode',self.start,mesh=False)
        else:
            # Reexecute the physical predecessor so training includes its
            # contact/solver history. No reset at the Anchor boundary.
            self.start=self.entry_acquirer()
            self.scene.begin_recording(self.out/'episode',mesh=False)
            self.metadata['entry_acquisition']='physical_predecessor_execution_each_episode'
        self.k=0;self.commands=[];self.previous_action=np.zeros(18);self.residuals=[];self.physical_prefix=None
        self.contact_run=0;self.preload_bounds=None
        self.acquisition_post_commands=None
        self.acquisition=(AcquiredGraspProposal(self.scene,self.anchor,self.data)
            if self.anchor.entry_contact=='free' and self.anchor.exit_contact=='grasped' else None)
        self.acquisition_nominal_command=np.r_[self.scene.command_pose,self.scene.hand_command,self.scene.command_twist]
        self.objective=WindowEvaluation(self.scene,self.anchor,self.data,self.start);self.failure=None
        self.metadata['lifting_reward_enabled']=self.objective.lifting
        self.metadata['lifting_initial_height_m']=self.objective.initial_tool_height
        self.last_observation=self.observation(self.start)
        return self.last_observation

    def command_precheck(self):return validate_commands(self.scene,np.asarray(self.commands),self.start)

    def continuation_proposal(self,*,executed_prefix=False):
        # Unknown future policy actions are an initialization only. Continue
        # the last residual to avoid jumping back to raw retarget after commit.
        residuals=np.asarray(self.residuals)
        if not np.any(residuals):controls=self.baseline.copy()
        else:
            residuals=np.concatenate((residuals,np.repeat(residuals[-1:],self.horizon-len(residuals),axis=0)))
            phase=np.clip((self.begin+np.arange(len(self.baseline)+1))/max(1,(self.anchor.core_start-self.anchor.synth_start)*self.tpf),0,1)
            controls=apply_residual(self.scene,self.baseline,self.start,residuals,self.tpf,phase**2*(3-2*phase),self.anchor)
        # Failed optimizer seeds are unadapted command proposals. Mixing in
        # the executed acquisition prefix makes the next PPO interpret the
        # already applied source-object lift as another pose residual.
        # Only a physical-prefix export or post-context compilation includes
        # executed drives, and zero actions must include them in that role too.
        if executed_prefix:
            count=min(len(self.commands),len(controls));controls[:count]=self.commands[:count]
        return controls

    def step(self,action):
        action=np.clip(np.asarray(action,dtype=float),-1,1)
        if action.shape!=(18,) or not np.isfinite(action).all():raise ValueError('Invalid floating PPO action')
        self.residuals.append(action*self.action_scales)
        end=(self.k+1)*self.tpf
        phase=np.clip((self.begin+np.arange(end+1))/max(1,(self.anchor.core_start-self.anchor.synth_start)*self.tpf),0,1)
        envelope=phase**2*(3-2*phase)
        # Exact zero policy keeps the original controls and their feedforward.
        candidate=(self.baseline[:end].copy() if not np.any(self.residuals) else
            apply_residual(self.scene,self.baseline[:end],self.start,np.asarray(self.residuals),self.tpf,envelope,self.anchor))
        self.objective.last_reward=0.
        try:
            if self.controller_handoff and self.k==0:
                from multipath.execution.wrench_handoff import segment_wrenches
                handoff_commands=self.baseline.copy();handoff_commands[:end]=candidate
                self.wrenches=segment_wrenches(self.scene,self.start,handoff_commands)
            for tick in range(self.k*self.tpf,end):
                c=candidate[tick];frame=self.anchor.synth_start+(self.begin+tick+1)/self.tpf
                if self.acquisition is not None and frame<=self.anchor.core_end:
                    c,self.acquisition_nominal_command=self.acquisition.command_with_residual(
                        frame,self.acquisition_reference[tick],c,self.acquisition_nominal_command)
                elif self.acquisition is not None and getattr(self.scene,'freeze_carry_core',False):
                    core_tick=round((self.anchor.core_end-self.anchor.synth_start)*self.tpf)-self.begin
                    if core_tick>0:
                        if self.acquisition_post_commands is None:
                            complete=self.continuation_proposal(executed_prefix=True)
                            complete[:len(self.commands)]=self.commands
                            complete=project_hand_commands(self.scene,complete,self.start,self.anchor)
                            self.acquisition_post_commands=complete[core_tick:]
                        # Stop the actually acquired core drive. Returning to
                        # a stationary pregrasp's nominal post-padding undid
                        # the very lift the policy was learning.
                        c=self.acquisition_post_commands[tick-core_tick].copy()
                c,self.preload_bounds=acquisition_preload(self.scene,self.anchor,c,self.preload_bounds)
                if self.controller_handoff:
                    if getattr(self.scene,'post_context_commands',None) is not None and frame>self.anchor.core_end:
                        if frame-self.fps*self.scene.dt==self.anchor.core_end:
                            from multipath.execution.wrench_handoff import segment_wrenches
                            complete=self.continuation_proposal();complete[:end]=candidate
                            self.post_wrenches=segment_wrenches(self.scene,self.scene.snapshot(),complete[tick:],transport=True)
                            self.post_begin=tick
                        self.scene.wrench_bias=self.post_wrenches[tick-self.post_begin]
                    else:self.scene.wrench_bias=self.wrenches[tick]
                self.scene.step_control(c,'LOCAL_PPO',frame);self.commands.append(c)
            self.objective.observe(frame,c);self.failure=self.objective.failure
        except (RuntimeError,ValueError) as error:self.failure={'source_frame':self.scene.ticks*self.scene.dt*self.fps,'reason':str(error)}
        self.k+=1;self.previous_action=action
        if self.failure is None and not self.objective.tracking_failures and self.k==min(20,self.horizon):
            self.physical_prefix={'state':self.scene.snapshot(),'trace':list(self.scene.records)}
        done=self.failure is not None or self.k==self.horizon
        if done:self.scene.save()
        obs=self.observation(self.scene.snapshot()) if self.failure is None else np.zeros_like(self.last_observation)
        self.last_observation=obs
        tracking_failure=next(iter(self.objective.tracking_failures),None)
        return obs,float(self.objective.last_reward),done,{'success':bool(done and self.failure is None and tracking_failure is None and len(self.commands)==len(self.baseline)),
            'failure':self.failure or tracking_failure,'tracking_failure_count':len(self.objective.tracking_failures),
            'source_steps':self.k,'evaluation_return':self.objective.evaluation_return,
            'reward_terms':dict(self.objective.last_reward_terms)}


def refine(scene,anchor,data,baseline,start,out,candidates=64,iterations=3,use_ppo=True,ppo_config=None):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);history=[];rng=np.random.default_rng(0)
    tpf=physics_ticks_per_frame(scene.manifest['fps'],scene.dt);chosen=baseline.copy();state=start;started=time.perf_counter()
    if len(baseline)!=(anchor.synth_end-anchor.synth_start)*tpf or not np.isclose(start['time_s']*scene.manifest['fps'],anchor.synth_start,atol=1e-8,rtol=0):
        raise ValueError('MPC baseline/entry does not match the padded source interval')
    prior_origin={}
    def ppo_env(*args):
        env=FloatingResidualEnv(*args,controller_handoff=True,acquisition_reference=replay)
        env.metadata.update(residual_center='best_physically_evaluated_local_command_proposal',
                            initialization_proposal=dict(prior_origin),initialization_is_acceptance=False)
        return env
    ppo=LocalPPOSolver(scene,anchor,data,out/'ppo',ppo_config or PPOConfig(),env_factory=ppo_env,allow_physical_prefix=True) if use_ppo else None
    def evaluate(control,boundary,begin,label,folder,object_relative=False,prefix_ticks=None,acquisition_reference=None):
        check=validate_commands(scene,control,boundary)
        if not check['feasible']:return -np.inf,False,{'precheck':check},None
        scene.begin_trial(folder,boundary,mesh=object_relative);objective=WindowEvaluation(scene,anchor,data,boundary)
        prefix=None
        proposal_control=np.array(control).copy()
        nominal_control=(proposal_control if acquisition_reference is None else np.asarray(acquisition_reference))
        nominal_previous=np.r_[scene.command_pose,scene.hand_command,scene.command_twist]
        executed_control=np.array(control).copy();held=None;acquired_frame=None;handoff=None;contact_run=0;executed=0;preload_bounds=None
        if object_relative and boundary['contact']['mode']=='grasped':
            held=grasp_drive_transform(boundary);preload_bounds=grasp_command_bounds(scene,boundary,anchor)
        acquisition=(AcquiredGraspProposal(scene,anchor,data)
            if anchor.entry_contact=='free' and anchor.exit_contact=='grasped' else None)
        try:
            for j,c in enumerate(control):
                frame=anchor.synth_start+(begin+j+1)/tpf
                if acquisition is not None and frame<=anchor.core_end:
                    c,nominal_previous=acquisition.command_with_residual(frame,nominal_control[j],c,nominal_previous)
                    held=acquisition.held
                    acquired_frame=acquisition.acquired_frame;preload_bounds=acquisition.preload_bounds
                    handoff=acquisition.handoff;contact_run=acquisition.contact_run;executed_control[j]=c
                    if frame==anchor.core_end and getattr(scene,'freeze_carry_core',False):
                        control[:j+1]=executed_control[:j+1]
                        control[j+1:]=project_hand_commands(scene,control,boundary,anchor)[j+1:]
                        executed_control[j+1:]=control[j+1:]
                elif object_relative:
                    measured=scene.snapshot()
                    contact_run=contact_run+1 if measured['contact']['mode']=='grasped' else 0
                    # One source frame of consecutive measured opposition is
                    # needed to initialize a grasp proposal, not an acceptance
                    # condition and not additional waiting/source time.
                    if held is None and frame>anchor.core_start and contact_run>=tpf:
                        held=np.asarray(measured['hand_object_transform']);acquired_frame=scene.ticks*scene.dt*scene.manifest['fps']
                        preload_bounds=grasp_command_bounds(scene,measured,anchor)
                    if (held is not None and anchor.entry_contact=='free'
                            and measured['contact']['mode']=='grasped' and scene.ticks%tpf==0):
                        # Use the acquired frame while fingers continue to
                        # close, rather than freezing the first rim contact.
                        # Only acquisition is adaptive; carry retention uses
                        # its original, fixed physically reached transform.
                        held=np.asarray(measured['hand_object_transform'])
                        low,high=grasp_command_bounds(scene,measured,anchor)
                        preload_bounds=(np.maximum(preload_bounds[0],low),np.minimum(preload_bounds[1],high))
                    if held is not None and frame<=anchor.core_end:
                        if handoff is None:
                            handoff=object_relative_handoff(scene,anchor,data,scene.ticks*scene.dt*scene.manifest['fps'],held)
                        c=object_relative_command(scene,anchor,data,frame,c,held,handoff)
                        low,high=preload_bounds;step=scene.hand_velocity*scene.dt
                        c[7:19]=np.clip(c[7:19],np.maximum(low,scene.hand_command-step),
                                       np.minimum(high,scene.hand_command+step));executed_control[j]=c
                        if frame==anchor.core_end and getattr(scene,'freeze_carry_core',False):
                            control[:j+1]=executed_control[:j+1]
                            control[j+1:]=project_hand_commands(scene,control,boundary,anchor)[j+1:]
                            executed_control[j+1:]=control[j+1:]
                        # Acquisition Anchor may transfer contact while closing.
                        # Its original core/object and exit-contact gates apply;
                        # persistent retention already applies to entry-grasped Anchors.
                if j==0:
                    from multipath.execution.wrench_handoff import segment_wrenches
                    handoff_controls=np.asarray(control).copy();handoff_controls[0]=c
                    wrenches=segment_wrenches(scene,boundary,handoff_controls)
                if getattr(scene,'post_context_commands',None) is not None and frame>anchor.core_end:
                    if frame-scene.dt*scene.manifest['fps']==anchor.core_end:
                        post_wrenches=segment_wrenches(scene,scene.snapshot(),control[j:],transport=True);post_begin=j
                    scene.wrench_bias=post_wrenches[j-post_begin]
                else:scene.wrench_bias=wrenches[j]
                c,preload_bounds=acquisition_preload(scene,anchor,c,preload_bounds)
                control[j]=c;executed_control[j]=c
                scene.step_control(c,label,frame);executed=j+1
                if (j+1)%tpf==0 and not objective.observe(frame,c):break
                if j+1==prefix_ticks and not objective.tracking_failures:
                    prefix={'state':scene.snapshot(),'trace':list(scene.records)}
        except (RuntimeError,ValueError) as error:objective.failure={'reason':str(error)};objective.total_return=-np.inf
        scene.save()
        if acquisition is not None:
            valid_prediction=(objective.failure is None and not objective.tracking_failures and executed==len(control))
            # A failed prediction is a nominal optimization seed, not a
            # hybrid of measured prefix and untouched stationary future.
            control[:]=executed_control if valid_prediction else proposal_control
            np.savez_compressed(Path(folder)/'executed_commands.npz',commands=executed_control[:executed])
            np.savez_compressed(Path(folder)/'proposal_commands.npz',commands=proposal_control)
        elif object_relative:
            # Complete the command proposal after an unsuccessful prediction.
            # These future commands are initialization, never physical evidence.
            if handoff is not None:
                for future in range(executed,len(control)):
                    frame=anchor.synth_start+(begin+future+1)/tpf
                    if frame>anchor.core_end and getattr(scene,'freeze_carry_core',False):
                        core_tick=round((anchor.core_end-anchor.synth_start)*tpf)-begin
                        if core_tick>0:
                            executed_control[core_tick:]=stopping_context(executed_control[core_tick-1],
                                (anchor.synth_end-anchor.core_end)*tpf,scene.dt)[:len(control)-core_tick]
                        break
                    executed_control[future]=object_relative_command(scene,anchor,data,frame,control[future],held,handoff)
                    executed_control[future,7:19]=np.clip(executed_control[future,7:19],*preload_bounds)
            control[:]=executed_control
            np.savez_compressed(Path(folder)/'commands.npz',commands=executed_control)
        tracking_failure=next(iter(objective.tracking_failures),None)
        return objective.total_return,objective.failure is None and tracking_failure is None and executed==len(control),{'failure':objective.failure or tracking_failure,'score':objective.total_return,
            'tracking_failure_count':len(objective.tracking_failures),
            'proposal':('acquisition_with_pose_finger_residual' if acquisition is not None else
                        'measured_grasp_object_relative' if object_relative else 'pose_residual'),
            'grasp_acquired_frame':acquired_frame,'target_handoff':handoff,'physics_steps_executed':executed,
            'prediction_complete':executed==len(control)},prefix
    def retain_prefix(prefix,commands,begin):
        if prefix is None:raise RefinementRequired('Successful prediction did not record its physical prefix')
        folder=out/f'committed_{begin}';folder.mkdir(parents=True,exist_ok=True)
        acquired=dict(prefix['state'])
        acquired['entry_provenance']=dict(acquired['entry_provenance'],trace=str(folder/'actual_trace.json'),
                                        acquisition='selected_physical_prediction_prefix')
        (folder/'actual_trace.json').write_text(json.dumps(prefix['trace']))
        (folder/'prefix_state.json').write_text(json.dumps(acquired,indent=2))
        np.savez_compressed(folder/'commands.npz',commands=commands[:len(prefix['trace'])*tpf])
        return acquired
    for begin in range(0,len(baseline),20*tpf):
        stop=min(begin+40*tpf,len(baseline));commit=min(20*tpf,len(baseline)-begin)
        replay=project_hand_commands(scene,chosen[begin:stop],state,anchor)
        score,valid,info,prefix=evaluate(replay,state,begin,'CHUNK_REPLAY',out/f'replay_{begin}',prefix_ticks=commit)
        best=replay if valid else None;best_score=score if valid else -np.inf
        best_state=retain_prefix(prefix,replay,begin) if valid else None
        seed=replay.copy();seed_score=score;prior_origin.clear();prior_origin.update(proposal='local_replay',score=float(score),failure=info.get('failure'))
        mean=np.zeros((len(replay)//tpf,18));scale=np.r_[np.full(3,.02),np.full(3,.2),np.full(12,.25)]
        phase=np.clip((begin+np.arange(len(replay)+1))/max(1,(anchor.core_start-anchor.synth_start)*tpf),0,1);envelope=phase**2*(3-2*phase)
        if not valid and anchor.exit_contact=='grasped' and anchor.entry_contact!='free':
            # First test the existing observed grasp-transform constraint as a
            # physical MPC proposal; the failed Replay contributes no state.
            proposal=replay.copy()
            value,ok,info,prefix=evaluate(proposal,state,begin,'OBJECT_RELATIVE_MPPI',out/f'object_relative_{begin}',object_relative=True,prefix_ticks=commit)
            history.append(dict(begin_tick=begin,candidate='measured_grasp',success=ok,**info))
            (out/'search.json').write_text(json.dumps(history,indent=2))
            print('FLOATING_OBJECT_RELATIVE',begin,'valid',ok,'failure',info.get('failure'),flush=True)
            if ok:
                best,best_score=proposal.copy(),value;best_state=retain_prefix(prefix,best,begin)
            if value>seed_score:
                seed,seed_score=proposal.copy(),value;prior_origin.update(proposal=info['proposal'],score=float(value),failure=info.get('failure'))
        if not valid and best is None:
            for iteration in range(iterations):
                samples=rng.normal(size=(candidates,*mean.shape))*scale*(.5**iteration)+mean;samples[0]=mean
                if anchor.entry_contact=='free' and anchor.exit_contact=='grasped' and iteration==0:
                    from multipath.xhand import RIGHT_XHAND_CLOSING_DIRECTION
                    # Independent per-frame finger noise repeatedly opened a
                    # transient grasp. Use two existing candidate slots for
                    # persistent closing offsets, sized by the same MPPI
                    # sigma. These are proposals, not controller state locks.
                    for candidate in range(1,min(candidates,3)):
                        samples[candidate]=mean
                        samples[candidate,:,6:]+=candidate*scale[6:]*np.asarray(RIGHT_XHAND_CLOSING_DIRECTION)
                    pinch=scene.manifest.get('retarget_details',{}).get(anchor.id,{}).get('pinch_proposal')
                    if pinch is not None:
                        gradient=np.asarray(pinch['gap_gradient_m_per_rad'])
                        if gradient.shape!=(2,12) or not np.isfinite(gradient).all():
                            raise ValueError('Invalid geometric pinch proposal')
                        # Local surface Jacobians convert a proposed contact
                        # load into existing joint-position drive offsets.
                        # These use two existing slots, not object forces or
                        # a contact acceptance rule. All limits and fresh
                        # padded/core replay requirements remain unchanged.
                        load=scene.physical_properties['tool_mass_kg']*(9.81+scene.cfg['planning']['acceleration_limits'][2])
                        offset=-gradient.sum(0)*load/scene.hand_drive_stiffness
                        for candidate,factor in [(3,.5),(4,1.)]:
                            if candidate>=candidates:break
                            samples[candidate]=mean
                            samples[candidate,:,6:]+=factor*offset
                returns=np.full(candidates,-np.inf)
                for i,residual in enumerate(samples):
                    control=replay.copy() if not np.any(residual) else apply_residual(scene,replay,state,residual,tpf,envelope,anchor)
                    contact_proposal=(iteration==0 and i in (3,4) and anchor.entry_contact=='free'
                        and anchor.exit_contact=='grasped' and scene.manifest.get('retarget_details',{}).get(anchor.id,{}).get('pinch_proposal') is not None)
                    folder=out/f'contact_preload_{begin}_{i}' if contact_proposal else out/'candidate'
                    returns[i],ok,info,prefix=evaluate(control,state,begin,'LOCAL_MPPI',folder,prefix_ticks=commit,acquisition_reference=replay)
                    if contact_proposal:info['initialization']='surface_Jacobian_finger_drive_preload'
                    history.append(dict(begin_tick=begin,iteration=iteration,candidate=i,success=ok,**info))
                    if ok and returns[i]>best_score:
                        best,best_score=control.copy(),returns[i];best_state=retain_prefix(prefix,best,begin)
                    if returns[i]>seed_score:
                        seed,seed_score=control.copy(),returns[i];prior_origin.update(proposal=info['proposal'],score=float(returns[i]),failure=info.get('failure'))
                updated=mppi_mean(samples,returns)
                if updated is not None:mean=updated
                (out/'search.json').write_text(json.dumps(history,indent=2))
                print('FLOATING_MPPI',begin,iteration,'valid',best is not None,'elapsed_s',round(time.perf_counter()-started,2),flush=True)
        if best is None and ppo is not None:
            best=ppo(seed,state,begin)
            if best is not None:
                best_state=retain_prefix(ppo.selected_prefix,best,begin)
            elif ppo.best_proposal is not None:
                seed=ppo.best_proposal.copy()
                prior_origin.update(proposal='best_failed_PPO_continuation',physical_acceptance=False)
        if best is None:
            methods='Replay/MPPI/PPO' if use_ppo else 'Replay/MPPI (PPO disabled for attribution)'
            error=RefinementRequired(f'Floating {methods} failed at source frame {anchor.synth_start+begin/tpf}')
            # Keep the already acquired prefix and best failed future proposal
            # for a continuous whole-Anchor physical replay. This is not a
            # validated trajectory or a state transition.
            proposal=chosen.copy();proposal[begin:stop]=seed
            error.controls=proposal
            raise error
        # Continue local development from the prefix actually reached inside the
        # selected physical prediction, rather than replaying it a second time.
        # This development restore is never final trajectory acceptance: a fresh
        # uninterrupted padded rollout and exported-core rollout are mandatory.
        # Retain the physically evaluated lookahead as the next MPC prior.
        # Reverting to raw Replay here can jump back after a grasp correction.
        state=best_state;chosen[begin:stop]=best
    return chosen
