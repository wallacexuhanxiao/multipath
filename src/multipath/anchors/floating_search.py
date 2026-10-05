"""Replay -> Cartesian MPPI -> small residual PPO for floating XHand.

Same source clock, core objective and contact evidence as the mounted path;
no arm IK, arm dynamic fitting, or full robot reference dependency.
"""
import json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from multipath.anchors.mpc import WindowEvaluation,mppi_mean,RefinementRequired
from multipath.anchors.floating import controls_from_poses,validate_commands,object_relative_command,object_relative_handoff,hand_command_boundary
from multipath.anchors.ppo import MountedResidualEnv,LocalPPOSolver,PPOConfig
from multipath.planning.timing import physics_ticks_per_frame
from multipath.validation.physics import object_tracking_parameters


def apply_residual(scene,baseline,start,residual,tpf,envelope):
    """Interpolate world-frame wrist residuals and enforce hand joint/speed bounds."""
    ticks=len(baseline);sample=np.arange(ticks+1)/tpf
    delta=np.vstack((np.zeros(18),residual));grid=np.arange(len(delta))
    interp=np.stack([np.interp(sample,grid,delta[:,j]) for j in range(18)],axis=1)
    interp*=np.asarray(envelope)[:,None]
    poses=np.vstack((start['wrist_xyz_wxyz'],baseline[:,:7]));hands=np.vstack((hand_command_boundary(start),baseline[:,7:19]))
    poses[:,:3]+=interp[:,:3]
    rotations=Rotation.from_rotvec(interp[:,3:6])*Rotation.from_quat(poses[:,[4,5,6,3]])
    poses[:,3:]=rotations.as_quat()[:,[3,0,1,2]];hands+=interp[:,6:]
    hands=np.clip(hands,scene.hand_limits[:,0],scene.hand_limits[:,1])
    for k in range(1,len(hands)):hands[k]=np.clip(hands[k],hands[k-1]-scene.hand_velocity*scene.dt,hands[k-1]+scene.hand_velocity*scene.dt)
    controls=controls_from_poses(poses,hands,scene.dt)
    # Appendix reproduction samples pose/finger residuals; it does not add
    # finite-differenced random pose noise to the wrist velocity feedforward.
    # Keep the nominal source/bridge command twist and let physical feedback
    # execute the sampled pose target. RELAXED planning is unaffected.
    controls[:,19:25]=baseline[:,19:25]
    return controls


class FloatingResidualEnv(MountedResidualEnv):
    def __init__(self,scene,anchor,data,baseline,start,begin,out):
        self.scene,self.anchor,self.data=scene,anchor,data
        self.baseline,self.start,self.begin=baseline.copy(),start,begin
        self.out=Path(out);self.out.mkdir(parents=True,exist_ok=True)
        self.fps=scene.manifest['fps'];self.tpf=physics_ticks_per_frame(self.fps,scene.dt);self.horizon=len(baseline)//self.tpf
        if self.horizon<1 or len(baseline)%self.tpf or not validate_commands(scene,baseline,start)['feasible']:
            raise ValueError('PPO requires feasible whole-source-frame local commands')
        if not np.isclose(start['time_s']*self.fps,anchor.synth_start+begin/self.tpf,atol=1e-8,rtol=0):
            raise ValueError('PPO entry does not match local source time')
        self.prior=baseline[self.tpf-1::self.tpf,:19].copy();self.hand_velocity=scene.hand_velocity;self.trial=0
        self.metadata={'anchor':anchor.id,'begin_tick':begin,'source_start_frame':anchor.synth_start+begin/self.tpf,
            'source_frames':self.horizon,'source_fps':self.fps,'physics_dt':scene.dt,'entry_origin':start.get('entry_origin'),
            'entry_provenance':start.get('entry_provenance'),'source_provenance':scene.manifest['provenance'],
            'action':'wrist_xyz_rotvec_and_12_fingers','action_scales':self.action_scales.tolist(),'residual_center':'local_floating_replay',
            'training_scene':'floating_PhysX','residual_velocity_feedforward':'nominal_replay_unchanged','objective':'Appendix weighted object tracking + thumb AND finger contact',
            'tracking_parameters':object_tracking_parameters(),
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
        self.trial+=1;self.scene.begin_trial(self.out/'episode',self.start,mesh=False)
        self.k=0;self.commands=[];self.previous_action=np.zeros(18);self.residuals=[]
        self.objective=WindowEvaluation(self.scene,self.anchor,self.data,self.start);self.failure=None
        self.last_observation=self.observation(self.start)
        return self.last_observation

    def command_precheck(self):return validate_commands(self.scene,np.asarray(self.commands),self.start)

    def step(self,action):
        action=np.clip(np.asarray(action,dtype=float),-1,1)
        if action.shape!=(18,) or not np.isfinite(action).all():raise ValueError('Invalid floating PPO action')
        self.residuals.append(action*self.action_scales)
        end=(self.k+1)*self.tpf
        phase=np.clip((self.begin+np.arange(end+1))/max(1,(self.anchor.core_start-self.anchor.synth_start)*self.tpf),0,1)
        envelope=phase**2*(3-2*phase)
        # Exact zero policy keeps the original controls and their feedforward.
        candidate=(self.baseline[:end].copy() if not np.any(self.residuals) else
            apply_residual(self.scene,self.baseline[:end],self.start,np.asarray(self.residuals),self.tpf,envelope))
        self.objective.last_reward=0.
        try:
            for tick in range(self.k*self.tpf,end):
                c=candidate[tick];frame=self.anchor.synth_start+(self.begin+tick+1)/self.tpf
                self.scene.step_control(c,'LOCAL_PPO',frame);self.commands.append(c)
            self.objective.observe(frame,c);self.failure=self.objective.failure
        except (RuntimeError,ValueError) as error:self.failure={'source_frame':self.scene.ticks*self.scene.dt*self.fps,'reason':str(error)}
        self.k+=1;self.previous_action=action
        done=self.failure is not None or self.k==self.horizon
        obs=self.observation(self.scene.snapshot()) if self.failure is None else np.zeros_like(self.last_observation)
        self.last_observation=obs
        return obs,float(self.objective.last_reward),done,{'success':bool(done and self.failure is None and len(self.commands)==len(self.baseline)),
            'failure':self.failure,'source_steps':self.k,'evaluation_return':self.objective.evaluation_return}


def refine(scene,anchor,data,baseline,start,out,candidates=64,iterations=3,use_ppo=True,ppo_config=None):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);history=[];rng=np.random.default_rng(0)
    tpf=physics_ticks_per_frame(scene.manifest['fps'],scene.dt);chosen=baseline.copy();state=start;started=time.perf_counter()
    if len(baseline)!=(anchor.synth_end-anchor.synth_start)*tpf or not np.isclose(start['time_s']*scene.manifest['fps'],anchor.synth_start,atol=1e-8,rtol=0):
        raise ValueError('MPC baseline/entry does not match the padded source interval')
    prior_origin={}
    def ppo_env(*args):
        env=FloatingResidualEnv(*args)
        env.metadata.update(residual_center='best_physically_evaluated_local_command_proposal',
                            initialization_proposal=dict(prior_origin),initialization_is_acceptance=False)
        return env
    ppo=LocalPPOSolver(scene,anchor,data,out/'ppo',ppo_config or PPOConfig(),env_factory=ppo_env) if use_ppo else None
    def evaluate(control,boundary,begin,label,folder,object_relative=False):
        check=validate_commands(scene,control,boundary)
        if not check['feasible']:return -np.inf,False,{'precheck':check}
        scene.begin_trial(folder,boundary,mesh=object_relative);objective=WindowEvaluation(scene,anchor,data,boundary)
        executed_control=np.array(control).copy();held=None;acquired_frame=None;handoff=None;contact_run=0;executed=0
        if object_relative and boundary['contact']['mode']=='grasped':held=np.asarray(boundary['hand_object_transform'])
        try:
            for j,c in enumerate(control):
                frame=anchor.synth_start+(begin+j+1)/tpf
                if object_relative:
                    measured=scene.snapshot()
                    contact_run=contact_run+1 if measured['contact']['mode']=='grasped' else 0
                    # One source frame of consecutive measured opposition is
                    # needed to initialize a grasp proposal, not an acceptance
                    # condition and not additional waiting/source time.
                    if held is None and frame>anchor.core_start and contact_run>=tpf:
                        held=np.asarray(measured['hand_object_transform']);acquired_frame=scene.ticks*scene.dt*scene.manifest['fps']
                    if held is not None:
                        if handoff is None:
                            handoff=object_relative_handoff(scene,anchor,data,scene.ticks*scene.dt*scene.manifest['fps'],held)
                        c=object_relative_command(scene,anchor,data,frame,c,held,handoff);executed_control[j]=c
                        # Acquisition Anchor may transfer contact while closing.
                        # Its original core/object and exit-contact gates apply;
                        # persistent retention already applies to entry-grasped Anchors.
                scene.step_control(c,label,frame);executed=j+1
                if (j+1)%tpf==0 and not objective.observe(frame,c):break
        except (RuntimeError,ValueError) as error:objective.failure={'reason':str(error)};objective.total_return=-np.inf
        scene.save()
        if object_relative:
            # Complete the command proposal after an unsuccessful prediction.
            # These future commands are initialization, never physical evidence.
            if handoff is not None:
                for future in range(executed,len(control)):
                    frame=anchor.synth_start+(begin+future+1)/tpf
                    executed_control[future]=object_relative_command(scene,anchor,data,frame,control[future],held,handoff)
            control[:]=executed_control
            np.savez_compressed(Path(folder)/'commands.npz',commands=executed_control)
        return objective.total_return,objective.failure is None and executed==len(control),{'failure':objective.failure,'score':objective.total_return,
            'proposal':'measured_grasp_object_relative' if object_relative else 'pose_residual',
            'grasp_acquired_frame':acquired_frame,'target_handoff':handoff,'physics_steps_executed':executed,
            'prediction_complete':executed==len(control)}
    for begin in range(0,len(baseline),20*tpf):
        stop=min(begin+40*tpf,len(baseline));commit=min(20*tpf,len(baseline)-begin);replay=chosen[begin:stop].copy()
        score,valid,info=evaluate(replay,state,begin,'CHUNK_REPLAY',out/f'replay_{begin}')
        best=replay if valid else None;best_score=score if valid else -np.inf
        seed=replay.copy();seed_score=score;prior_origin.clear();prior_origin.update(proposal='local_replay',score=float(score),failure=info.get('failure'))
        mean=np.zeros((len(replay)//tpf,18));scale=np.r_[np.full(3,.02),np.full(3,.2),np.full(12,.25)]
        phase=np.clip((begin+np.arange(len(replay)+1))/max(1,(anchor.core_start-anchor.synth_start)*tpf),0,1);envelope=phase**2*(3-2*phase)
        if not valid and anchor.exit_contact=='grasped':
            # First test the existing observed grasp-transform constraint as a
            # physical MPC proposal; the failed Replay contributes no state.
            proposal=replay.copy()
            value,ok,info=evaluate(proposal,state,begin,'OBJECT_RELATIVE_MPPI',out/f'object_relative_{begin}',object_relative=True)
            history.append(dict(begin_tick=begin,candidate='measured_grasp',success=ok,**info))
            (out/'search.json').write_text(json.dumps(history,indent=2))
            print('FLOATING_OBJECT_RELATIVE',begin,'valid',ok,'failure',info.get('failure'),flush=True)
            if ok:best,best_score=proposal.copy(),value
            if value>seed_score:
                seed,seed_score=proposal.copy(),value;prior_origin.update(proposal=info['proposal'],score=float(value),failure=info.get('failure'))
        if not valid and best is None:
            for iteration in range(iterations):
                samples=rng.normal(size=(candidates,*mean.shape))*scale*(.5**iteration)+mean;samples[0]=mean
                returns=np.full(candidates,-np.inf)
                for i,residual in enumerate(samples):
                    control=replay.copy() if not np.any(residual) else apply_residual(scene,replay,state,residual,tpf,envelope)
                    returns[i],ok,info=evaluate(control,state,begin,'LOCAL_MPPI',out/'candidate')
                    history.append(dict(begin_tick=begin,iteration=iteration,candidate=i,success=ok,**info))
                    if ok and returns[i]>best_score:best,best_score=control.copy(),returns[i]
                    if returns[i]>seed_score:
                        seed,seed_score=control.copy(),returns[i];prior_origin.update(proposal=info['proposal'],score=float(returns[i]),failure=info.get('failure'))
                updated=mppi_mean(samples,returns)
                if updated is not None:mean=updated
                (out/'search.json').write_text(json.dumps(history,indent=2))
                print('FLOATING_MPPI',begin,iteration,'valid',best is not None,'elapsed_s',round(time.perf_counter()-started,2),flush=True)
        if best is None and ppo is not None:best=ppo(seed,state,begin)
        if best is None:
            methods='Replay/MPPI/PPO' if use_ppo else 'Replay/MPPI (PPO disabled for attribution)'
            raise RefinementRequired(f'Floating {methods} failed at source frame {anchor.synth_start+begin/tpf}')
        _,ok,info=evaluate(best[:commit],state,begin,'LOCAL_COMMIT',out/f'committed_{begin}')
        if not ok:raise RefinementRequired(f'Floating selected controls failed commit: {info}')
        # Retain the physically evaluated lookahead as the next MPC prior.
        # Reverting to raw Replay here can jump back after a grasp correction.
        state=scene.snapshot();chosen[begin:stop]=best
    return chosen
