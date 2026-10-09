"""Small per-Anchor residual PPO on the actual mounted articulation.

Training restores only a physically acquired development snapshot. A policy
observes actual state each source frame; it never writes robot/object state.
Selected controls still require fresh padded and exported-core execution.
"""
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from multipath.anchors.control_check import control_feasibility
from multipath.anchors.mpc import WindowEvaluation, fitted_cartesian_prior
from multipath.anchors.synthesis import hand_limits_from_urdf
from multipath.anchors.residual_control import project_arm_command
from multipath.validation.physics import object_tracking_parameters,object_optimization_parameters


@dataclass(frozen=True)
class PPOConfig:
    iterations: int = 60
    episodes_per_update: int = 4
    max_seconds: float = 480.
    evaluate_every: int = 5
    seed: int = 0
    exploration_final_scale: float = .25

    def __post_init__(self):
        if (min(self.iterations,self.episodes_per_update,self.evaluate_every) < 1 or self.max_seconds <= 0
                or not 0 < self.exploration_final_scale <= 1):
            raise ValueError('PPO needs a positive bounded training budget')


class MountedResidualEnv:
    """One serial PhysX training environment at the fixed 30 Hz source clock."""
    action_dim = 18
    action_scales = np.r_[np.full(3,.02),np.full(3,.2),np.full(12,.2)]

    def __init__(self,scene,anchor,data,baseline,start,begin,out):
        self.scene,self.anchor,self.data = scene,anchor,data
        self.baseline,self.start,self.begin = baseline.copy(),start,begin
        self.out=Path(out);self.out.mkdir(parents=True,exist_ok=True)
        self.fps=scene.manifest['fps'];self.tpf=round(1/(self.fps*scene.dt))
        self.horizon=len(baseline)//self.tpf
        if len(baseline)%self.tpf or self.horizon < 1:
            raise ValueError('PPO window must contain whole source frames')
        self.prior=fitted_cartesian_prior(scene,baseline,self.tpf)
        all_limits=np.asarray(scene.robot._articulation_view.get_dof_limits()).reshape(-1,2)
        self.limits=all_limits[np.r_[scene.arm,scene.fingers]]
        self.hand_velocity=hand_limits_from_urdf(scene)
        self.trial=0
        self.metadata={'anchor':anchor.id,'begin_tick':begin,'source_start_frame':anchor.synth_start+begin/self.tpf,
            'source_frames':self.horizon,'source_fps':self.fps,'physics_dt':scene.dt,
            'entry_origin':start.get('entry_origin'),'entry_provenance':start.get('entry_provenance'),
            'source_provenance':scene.manifest['provenance'],'action':'wrist_xyz_rotvec_and_12_fingers',
            'action_scales':self.action_scales.tolist(),'residual_center':'fitted_local_replay',
            'dynamics_projection':'command_space_only','training_scene':'mounted_PhysX',
            'objective':'Appendix C.3+C.7+C.8',
            'optimization_parameters':object_optimization_parameters(),
            'early_termination':'C.4 weighted error > C',
            'terminal_reward':'same_tracking_formula_no_added_failure_penalty',
            'evaluation_reward':'object_tracking_only_C.9',
            'tracking_parameters':object_tracking_parameters(),
            'clock_stretched':False,'state_resets_within_episode':0,'domain_randomization':False}

    def observation(self,state):
        i=min(self.k,self.horizon-1)
        frame=self.anchor.synth_start+self.begin/self.tpf+i+1
        idx=min(max(round(frame)-self.anchor.synth_start,0),len(self.data['tool_pose'])-1)
        tool_ref=self.data['tool_pose'][idx]
        expected=np.r_[tool_ref[:3,3],Rotation.from_matrix(tool_ref[:3,:3]).as_quat()[[3,0,1,2]]]
        relative=np.asarray(state['hand_object_transform'])
        rel=np.r_[relative[:3,3],Rotation.from_matrix(relative[:3,:3]).as_quat()[[3,0,1,2]]]
        obs=np.r_[np.asarray(state['arm_q'])/3.,np.asarray(state['arm_dq'])/self.scene.velocity,
            state['hand_q'],np.asarray(state['hand_dq'])/self.hand_velocity,
            state['wrist_xyz_wxyz'],state['objects']['tool']['pose'],state['objects']['tool']['velocity'],
            state['objects']['target']['pose'],rel,self.prior[i],expected,
            self.k/self.horizon,self.previous_action]
        if not np.isfinite(obs).all():raise RuntimeError('Non-finite PPO observation')
        return obs.astype(np.float32)

    def reset(self):
        self.trial+=1
        # Reuse one scratch directory; save selected/best traces explicitly.
        self.scene.begin_trial(self.out/'episode',self.start,mesh=False)
        self.k=0;self.commands=[];self.previous_action=np.zeros(18)
        self.q=np.r_[self.start['arm_q'],self.start['hand_q']].astype(float)
        self.dq=np.asarray(self.start['arm_dq'],dtype=float)
        self.previous_delta=np.zeros(19)
        self.ik_seed=np.asarray(self.start['arm_q']).copy()
        self.objective=WindowEvaluation(self.scene,self.anchor,self.data,self.start)
        self.metadata['lifting_reward_enabled']=self.objective.lifting
        self.metadata['lifting_initial_height_m']=self.objective.initial_tool_height
        self.failure=None
        return self.observation(self.start)

    def step(self,action):
        action=np.asarray(action,dtype=float)
        if action.shape!=(18,) or not np.isfinite(action).all():raise ValueError('Invalid residual PPO action')
        action=np.clip(action,-1,1)
        desired=self.prior[self.k].copy();residual=action*self.action_scales
        desired[:3]+=residual[:3]
        rotation=Rotation.from_rotvec(residual[3:6])*Rotation.from_quat(desired[[4,5,6,3]])
        desired[3:7]=rotation.as_quat()[[3,0,1,2]]
        desired[7:]+=residual[6:]
        end=(self.k+1)*self.tpf
        reference=self.baseline[end-1,:19]
        self.objective.last_reward=0.
        try:
            if not np.any(action):
                delta=np.zeros(19)
            else:
                arm=self.scene.solve(desired[:7],self.ik_seed)
                self.ik_seed=arm.copy()
                delta=np.r_[arm,desired[7:]]-reference
            for sub in range(self.tpf):
                tick=self.k*self.tpf+sub
                alpha=(sub+1)/self.tpf
                frame=self.anchor.synth_start+(self.begin+tick+1)/self.tpf
                # Allocate residual authority inside existing padding; no wait
                # and no time added. Zero residual preserves the fitted replay.
                context=max(1,self.anchor.core_start-self.anchor.synth_start)
                phase=np.clip((self.begin+tick)/self.tpf/context,0,1)
                envelope=phase*phase*(3-2*phase)
                proposed=self.baseline[tick,:19]+envelope*((1-alpha)*self.previous_delta+alpha*delta)
                dt=self.scene.dt
                vmax=self.scene.velocity;amax=np.broadcast_to(self.scene.acceleration,(7,))
                arm,low,high=project_arm_command(proposed[:7],self.q[:7],self.dq,
                                                 dt,self.limits[:7],vmax,amax)
                hand_low=np.maximum(self.limits[7:,0],self.q[7:]-self.hand_velocity*dt)
                hand_high=np.minimum(self.limits[7:,1],self.q[7:]+self.hand_velocity*dt)
                if np.any(low>high) or np.any(hand_low>hand_high):
                    raise RuntimeError('Residual commands cannot satisfy local actuator boundary')
                hand=np.clip(proposed[7:],hand_low,hand_high)
                # Preserve measured initial velocity for the very first step.
                if tick==0:
                    arm=np.asarray(self.start['arm_q'])+np.asarray(self.start['arm_dq'])*dt
                    hand=np.asarray(self.start['hand_q'])+np.asarray(self.start['hand_dq'])*dt
                    if np.any(arm<low-1e-6) or np.any(arm>high+1e-6) or np.any(hand<hand_low-1e-6) or np.any(hand>hand_high+1e-6):
                        raise RuntimeError('Measured entry velocity has no feasible first control step')
                velocity=(arm-self.q[:7])/dt
                command=np.r_[arm,hand,velocity]
                self.scene.step(arm,hand,'LOCAL_PPO',frame,velocity)
                self.commands.append(command)
                self.q=np.r_[arm,hand];self.dq=velocity
            self.objective.observe(frame,command)
            self.failure=self.objective.failure
        except (RuntimeError,ValueError) as error:
            self.failure={'source_frame':self.scene.ticks*self.scene.dt*self.fps,
                          'reason':str(error)}
        self.k+=1;self.previous_action=action
        if self.failure is None:self.previous_delta=delta
        done=self.failure is not None or self.k==self.horizon
        # The failed tracking frame uses the same C-error + contact formula.
        # Execution failures before observation receive zero, no bespoke penalty.
        reward=self.objective.last_reward
        observation=self.observation(self.scene.snapshot()) if self.failure is None else np.zeros_like(self.observation(self.start))
        success=bool(done and self.failure is None and len(self.commands)==len(self.baseline))
        return observation,float(reward),done,{'success':success,'failure':self.failure,'source_steps':self.k,
            'evaluation_return':self.objective.evaluation_return,
            'reward_terms':dict(self.objective.last_reward_terms)}

    def command_precheck(self):
        return control_feasibility(np.asarray(self.commands),np.asarray(self.start['arm_q']),np.asarray(self.start['arm_dq']),
            self.scene.dt,self.limits,self.scene.velocity,self.scene.acceleration,
            start_hand_q=np.asarray(self.start['hand_q']),hand_velocity_limits=self.hand_velocity)

    def save_evaluation(self,folder,result):
        folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(folder/'commands.npz',commands=np.asarray(self.commands).reshape(-1,self.baseline.shape[1]))
        (folder/'actual_trace.json').write_text(json.dumps(self.scene.records))
        (folder/'evaluation.json').write_text(json.dumps(result,indent=2))


class LocalPPOSolver:
    """A small fresh model per failed window, sharing one wall-clock budget."""
    def __init__(self,scene,anchor,data,out,config=None,env_factory=None,accept_controls=None,allow_physical_prefix=False):
        self.scene,self.anchor,self.data=scene,anchor,data
        self.out=Path(out);self.out.mkdir(parents=True,exist_ok=True)
        self.config=config or PPOConfig();self.deadline=None;self.reports=[]
        self.env_factory=env_factory or MountedResidualEnv
        self.accept_controls=accept_controls
        self.allow_physical_prefix=allow_physical_prefix

    def __call__(self,baseline,start,begin):
        self.selected_prefix=None
        self.best_proposal=None;proposal_rank=None
        from multipath.anchors.ppo_model import SmallResidualPPO
        cfg=self.config
        if self.deadline is None:self.deadline=time.monotonic()+cfg.max_seconds
        folder=self.out/f'chunk_{begin:04d}_attempt_{len(self.reports):02d}'
        env=self.env_factory(self.scene,self.anchor,self.data,baseline,start,begin,folder)
        obs=env.reset();learner=SmallResidualPPO(len(obs),env.action_dim,seed=cfg.seed)
        report=dict(env.metadata,config=asdict(cfg),network_hidden=[64,64],
            parameters=learner.parameter_count,status='training',iterations=0,episodes=0,history=[],success=False,
            commit_requires_full_prediction=not self.allow_physical_prefix,
            residual_noise_schedule={'kind':'linear_multiplier_of_learned_Gaussian_std','initial':1.,
                'final':cfg.exploration_final_scale,'shared_across_sources':True,
                'numeric_provenance':'engineering choice; Appendix specifies shared schedule, not its values'})
        self.reports.append(report)
        (folder/'entry.json').write_text(json.dumps(start,indent=2))
        np.savez_compressed(folder/'baseline.npz',commands=baseline)
        started=time.monotonic();best_return=-np.inf

        def write():
            report['elapsed_s']=time.monotonic()-started
            (folder/'training_result.json').write_text(json.dumps(report,indent=2))
            (self.out/'rl_result.json').write_text(json.dumps({'windows':self.reports},indent=2))

        def rollout(deterministic):
            nonlocal proposal_rank
            obs=env.reset();episode=[];total=0.
            for _ in range(env.horizon):
                action,info=learner.act(obs,deterministic=deterministic)
                nxt,reward,done,result=env.step(action)
                episode.append(dict(obs=obs,latent=info['latent'],log_prob=info['log_prob'],
                                    value=info['value'],reward=reward,done=done))
                obs=nxt;total+=reward
                if done:break
            result=dict(result,return_value=total,deterministic=deterministic)
            if result['success']:
                check=env.command_precheck()
                result['command_precheck']=check;result['success']=check['feasible']
            if hasattr(env,'continuation_proposal'):
                rank=(result['source_steps'],result['evaluation_return'])
                if proposal_rank is None or rank>proposal_rank:
                    self.best_proposal=env.continuation_proposal().copy();proposal_rank=rank
                    np.savez_compressed(folder/'best_proposal.npz',commands=self.best_proposal)
                    report['best_proposal']=dict(source_steps=rank[0],evaluation_return=rank[1],
                        physical_acceptance=False,role='initialization_for_fresh_local_replay')
            report['episodes']+=1
            return episode,result

        def accepted(commands):
            if self.accept_controls is None:return True
            ok=bool(self.accept_controls(commands))
            report.setdefault('fresh_replay_acceptance',[]).append({'success':ok})
            write()
            return ok

        def physical_prefix(result):
            # A failed future prediction does not invalidate a prefix that
            # passed every observed physical/core gate. Use it for local MPC
            # development only; the final whole padded/core Replay is unchanged.
            prefix=getattr(env,'physical_prefix',None)
            if not self.allow_physical_prefix or prefix is None or result['success']:
                return None
            commands=env.continuation_proposal(executed_prefix=True)
            count=len(prefix['trace'])*env.tpf
            from multipath.anchors.floating import validate_commands
            if not validate_commands(self.scene,commands,start)['feasible']:return None
            selected=folder/'physical_prefix';selected.mkdir(exist_ok=True)
            (selected/'actual_trace.json').write_text(json.dumps(prefix['trace']))
            (selected/'state.json').write_text(json.dumps(prefix['state'],indent=2))
            np.savez_compressed(selected/'commands.npz',commands=commands[:count])
            report.update(status='physical_prefix_selected',success=False,
                          selection='local_development_prefix; fresh_padded_and_core_required',
                          committed_source_frames=count//env.tpf,future_prediction_failure=result.get('failure'))
            self.selected_prefix=prefix;write()
            learner.save(folder/'prefix_policy.pt',env.metadata)
            return commands

        # Confirm that the installed zero-mean policy really runs the baseline.
        _,zero=rollout(True);report['zero_policy']=zero;env.save_evaluation(folder/'zero_policy',zero)
        write();print('LOCAL_PPO_BEGIN',begin,'parameters',learner.parameter_count,'zero',zero,flush=True)
        if zero['success'] and accepted(np.asarray(env.commands).copy()):
            report.update(success=True,status='zero_policy_passed');write()
            learner.save(folder/'policy.pt',env.metadata)
            self.selected_prefix=getattr(env,'physical_prefix',None)
            return np.asarray(env.commands).copy()
        prefix=physical_prefix(zero)
        if prefix is not None:return prefix
        try:
            for iteration in range(cfg.iterations):
                if time.monotonic()>=self.deadline:break
                noise_scale=1.-(1.-cfg.exploration_final_scale)*iteration/max(1,cfg.iterations-1)
                learner.set_exploration_scale(noise_scale)
                episodes=[];results=[]
                for _ in range(cfg.episodes_per_update):
                    if time.monotonic()>=self.deadline:break
                    episode,result=rollout(False);episodes.append(episode);results.append(result)
                    prefix=physical_prefix(result)
                    if prefix is not None:return prefix
                    if result['success']:
                        # We synthesize one executable command trajectory.
                        # A physically valid sampled episode is useful even
                        # when the mean policy has not converged. Subsequent
                        # commit, padded Replay and core Replay still validate it.
                        commands=np.asarray(env.commands).copy()
                        env.save_evaluation(folder/'sampled_candidate',result)
                        prefix=getattr(env,'physical_prefix',None)
                        if not accepted(commands):continue
                        report.update(success=True,status='sampled_controls_passed',
                            iterations=iteration+1,sampled_candidate=result,
                            policy_deterministic_success=False,
                            selection='measured_sampled_commands; fresh_Replay_required')
                        learner.save(folder/'candidate_policy.pt',env.metadata)
                        write();print('LOCAL_PPO_SAMPLED_CONTROLS_PASSED',begin,iteration+1,flush=True)
                        self.selected_prefix=prefix
                        return commands
                if not episodes:break
                metrics=learner.update(episodes)
                row=dict(iteration=iteration+1,mean_return=float(np.mean([r['return_value'] for r in results])),
                    max_source_steps=max(r['source_steps'] for r in results),training_successes=sum(r['success'] for r in results),
                    exploration_scale=noise_scale,**metrics)
                from collections import Counter
                row['failure_counts']=dict(Counter((r.get('failure') or {}).get('reason','passed').split(':')[0] for r in results))
                report['iterations']=iteration+1;report['history'].append(row)
                print('LOCAL_PPO_UPDATE',begin,row,flush=True);write()
                if (iteration+1)%cfg.evaluate_every==0 or iteration+1==cfg.iterations or time.monotonic()>=self.deadline:
                    _,evaluation=rollout(True);row['evaluation']=evaluation
                    prefix=physical_prefix(evaluation)
                    if prefix is not None:return prefix
                    if evaluation['evaluation_return']>best_return:
                        best_return=evaluation['evaluation_return'];report['best_evaluation']=evaluation
                        learner.save(folder/'best_policy.pt',env.metadata)
                        env.save_evaluation(folder/'best_evaluation',evaluation)
                    if evaluation['success'] and accepted(np.asarray(env.commands).copy()):
                        report.update(success=True,status='window_passed');write()
                        learner.save(folder/'policy.pt',env.metadata)
                        self.selected_prefix=getattr(env,'physical_prefix',None)
                        return np.asarray(env.commands).copy()
            report['status']='budget_exhausted';write()
            return None
        finally:
            learner.save(folder/'last_policy.pt',env.metadata)
            write()
