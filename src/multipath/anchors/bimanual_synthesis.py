"""Primary-hand local MPC/PPO with an actually executed companion program."""
import json
from pathlib import Path
import numpy as np


def refine_primary(scene,anchor,data,baseline,companions,output,*,baseline_commands=None,seconds=240.,frozen_prefix_ticks=None):
    from multipath.anchors.mpc import WindowEvaluation,RefinementRequired
    from multipath.anchors.floating_search import refine,FloatingResidualEnv
    from multipath.anchors.ppo import LocalPPOSolver,PPOConfig
    from multipath.anchors.floating import project_hand_commands
    from multipath.execution.wrench_handoff import segment_wrenches
    out=Path(output);out.mkdir(parents=True,exist_ok=True);tpf=scene.tpf
    begin=(anchor.core_start-anchor.synth_start)*tpf
    frozen=begin if frozen_prefix_ticks is None else max(begin,int(frozen_prefix_ticks))
    if frozen>=len(baseline):raise ValueError('No local controls remain after the physically acquired prefix')
    prefix={key:np.stack([step[key] for step in scene.executed_controls[:anchor.synth_start*tpf]])
        for key in ('right','right_wrenches',*[key for side in companions for key in (side,side+'_wrenches')])}
    if len(prefix['right'])!=anchor.synth_start*tpf:raise ValueError('Missing physically executed predecessor')
    # Prefix inputs are measured-rollout controller records, not a robotized
    # full-video reference. Reexecute them from the actual initial scene.
    memory={};attempts=[];accepted={}
    properties=('frozen_synthesis_prefix_ticks','compiled_local_command_reference','freeze_carry_core',
                'bridge_exit_velocity_constraints')
    old={key:getattr(scene,key,None) for key in properties}
    scene.frozen_synthesis_prefix_ticks=frozen
    scene.compiled_local_command_reference=True;scene.freeze_carry_core=anchor.exit_contact=='grasped'
    # A valid local grasp whose actual exit speed exceeds the next planner's
    # nominal budget is not a usable canonical boundary. No hardware rating
    # or acceleration acceptance threshold is introduced here.
    scene.bridge_exit_velocity_constraints=anchor.exit_contact=='grasped'

    def acquire(folder):
        scene.supporting_control_provider=None;scene.control_wrench_provider=None
        scene.begin_initial_trial(folder,mesh=False)
        for k in range(anchor.synth_start*tpf):
            scene.wrench_bias=prefix['right_wrenches'][k]
            for side in companions:scene.supporting_hands[side].wrench_bias=prefix[side+'_wrenches'][k]
            scene.step_control(prefix['right'][k],'PHYSICAL_PREDECESSOR',(k+1)/tpf,
                supporting_commands={side:prefix[side][k] for side in companions})
        memory.clear()
        memory['companions']={side:segment_wrenches(scene.supporting_hands[side],
            scene.supporting_hands[side].snapshot(),value['commands']) for side,value in companions.items()}
        scene.supporting_control_provider=companion_control
        scene.control_wrench_provider=primary_wrench
        scene.save()
        return scene.snapshot()

    def companion_control(frame):
        k=round(frame*tpf)-1-anchor.synth_start*tpf
        return {side:(value['commands'][k],memory['companions'][side][k]) for side,value in companions.items()}

    def primary_wrench(frame,command):
        k=round(frame*tpf)-1-anchor.synth_start*tpf
        if k<begin:
            # Keep the already reached cuRobo prefix through the core entry.
            memory.pop('primary',None)
            return prefix_full_wrenches[k]
        if 'primary' not in memory:
            memory['primary']=segment_wrenches(scene,scene.snapshot(),baseline[begin:],
                baseline_commands=baseline_commands)
        return memory['primary'][k-begin]

    # Capture the complete incoming prefix before any local trial reset.
    prefix_full_wrenches=np.stack([step['right_wrenches'] for step in
        scene.executed_controls[anchor.synth_start*tpf:anchor.core_start*tpf]])
    if len(prefix_full_wrenches)!=begin:raise ValueError('Missing physically reached core context')

    def fresh(candidate):
        candidate=np.asarray(candidate).copy();candidate[:frozen]=baseline[:frozen]
        start=acquire(out/'predecessor');folder=out/'fresh_padded'/f'candidate_{len(attempts):03d}'
        scene.begin_recording(folder,mesh=True);scene.capture('SYNTHESIS_CONTEXT',anchor.synth_start)
        checks={'right':WindowEvaluation(scene,anchor,data,start)}
        for side,value in companions.items():
            checks[side]=WindowEvaluation(scene.supporting_hands[side],value['anchor'],value['data'],
                scene.supporting_hands[side].snapshot())
        failure=None
        for k,c in enumerate(candidate):
            frame=anchor.synth_start+(k+1)/tpf
            scene.step_control(c,'BIMANUAL_PADDED',frame)
            if (k+1)%tpf==0:
                for side,check in checks.items():
                    command=c if side=='right' else companions[side]['commands'][k]
                    if not check.observe(frame,command) and failure is None:failure=dict(check.failure,hand=side)
                if failure:break
        result=dict(success=failure is None and scene.ticks==anchor.synth_end*tpf,
            failure=failure,actual_end_frame=scene.ticks/tpf,state_resets_within_rollout=0)
        attempts.append(result);scene.save()
        if result['success']:
            accepted.update(commands=np.asarray(candidate).copy(),validation=result,
                primary_wrenches=np.stack([s['right_wrenches'] for s in scene.executed_controls]),
                companion_wrenches={side:np.stack([s[side+'_wrenches'] for s in scene.executed_controls])
                                    for side in companions})
        print('BIMANUAL_PRIMARY_FRESH',len(attempts),result,flush=True)
        return result['success']

    report=dict(source='oracle',source_provenance=scene.manifest['provenance'],
        frozen_physical_context_ticks=frozen,source_time_stretch=False,success=False)
    try:
        start=acquire(out/'mpc_entry');proposal=project_hand_commands(scene,baseline,start,anchor)
        try:proposal=refine(scene,anchor,data,proposal,start,out/'mpc',candidates=16,iterations=2,use_ppo=False)
        except RefinementRequired as error:
            report['mpc_failure']=str(error)
            if hasattr(error,'controls'):proposal=project_hand_commands(scene,error.controls,start,anchor)
        if not fresh(proposal):
            def environment(*values):
                return FloatingResidualEnv(*values,entry_acquirer=lambda:acquire(out/'ppo_entry'),controller_handoff=False)
            solver=LocalPPOSolver(scene,anchor,data,out/'ppo',PPOConfig(iterations=60,max_seconds=seconds),
                env_factory=environment,accept_controls=fresh)
            # Keep the real Replay reference rather than the rejected search
            # candidate. PPO samples a_ref + delta with Appendix C rewards.
            if solver(project_hand_commands(scene,baseline,start,anchor),start,0) is None:
                raise RuntimeError('Primary Replay/MPC/residual PPO failed fresh joint physical validation')
        report.update(success=True,attempts=attempts)
        (out/'report.json').write_text(json.dumps(report,indent=2));accepted['report']=report
        return accepted
    except Exception as error:
        report.update(success=False,error=f'{type(error).__name__}: {error}',attempts=attempts)
        (out/'report.json').write_text(json.dumps(report,indent=2))
        raise
    finally:
        scene.supporting_control_provider=None;scene.control_wrench_provider=None
        for key,value in old.items():
            if value is None:delattr(scene,key)
            else:setattr(scene,key,value)
