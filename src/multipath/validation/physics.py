import numpy as np
from scipy.spatial.transform import Rotation

OBJECT_POSITION_TOLERANCE_M = .05
OBJECT_ROTATION_TOLERANCE_RAD = float(np.deg2rad(30.))


def object_tracking_parameters():
    return {'C':1., 'lambda_p':1/OBJECT_POSITION_TOLERANCE_M**2,
            'lambda_R':1/OBJECT_ROTATION_TOLERANCE_RAD**2,
            'position_tolerance_m':OBJECT_POSITION_TOLERANCE_M,
            'rotation_tolerance_rad':OBJECT_ROTATION_TOLERANCE_RAD,
            'acceptance':'Appendix_C.4_weighted_error_le_C',
            'parameter_provenance':'shared project tolerances; numeric TACO weights not specified in the paper'}


def object_optimization_parameters():
    """Shared TACO C.7/C.8 coefficients, not action-specific tuning.

    The paper specifies these terms, but not their numeric TACO coefficients.
    Retain the existing contact bonus. A half-tracking-gradient lifting scale
    encourages lifting without outweighing position tracking past the target.
    """
    return {'contact_bonus':2.,'lifting_scale_per_m':.5/OBJECT_POSITION_TOLERANCE_M,
            'human_mimic_scale':0.,'action_smoothness_scale':0.,
            'domain_randomization':False,
            'parameter_provenance':'shared engineering coefficients; paper does not give numeric TACO values'}


def object_optimization_reward(tracking_reward,contact,lift_m=0.,*,lifting=False):
    """Appendix C.3+C.7+C.8; evaluation keeps C.3 alone."""
    p=object_optimization_parameters()
    opposition=bool(contact.get('thumb_contact',False) and contact.get('other_finger_contact',False))
    contact_reward=p['contact_bonus']*opposition
    lifting_reward=p['lifting_scale_per_m']*float(lift_m) if lifting else 0.
    return {'object_tracking':float(tracking_reward),'contact':float(contact_reward),
            'lifting':float(lifting_reward),
            'total':float(tracking_reward+contact_reward+lifting_reward)}


def retention_errors(state, held_transform):
    delta=np.linalg.inv(held_transform)@np.asarray(state['hand_object_transform'])
    return {'position_m':float(np.linalg.norm(delta[:3,3])),
            'rotation_rad':float(Rotation.from_matrix(delta[:3,:3]).magnitude()),
            'contact_retained':state['contact']['mode']=='grasped'}


def check_retention(state,held_transform,*,require_opposition=True):
    """Retain geometry every physics step, opposition at control boundaries.

    Between source control steps native finger contact is still required.
    A one-step thumb-contact dropout is recorded, never rewritten as contact.
    """
    errors=retention_errors(state,held_transform)
    if not require_opposition:
        errors['opposition_observed']=errors['contact_retained']
        errors['contact_retained']=bool(state['contact'].get('touching_links',0)>0)
        errors['contact_requirement']='native_touch_between_source_control_boundaries'
    if (errors['position_m']>OBJECT_POSITION_TOLERANCE_M or
            errors['rotation_rad']>OBJECT_ROTATION_TOLERANCE_RAD or not errors['contact_retained']):
        raise RuntimeError(f'Persistent grasp constraint failed: {errors}')
    return errors


def check_relaxed_tool_target_contact(scene,contact_steps_at_entry):
    """Native contact must not silently push the obstacle during transport.

    Triangle overlap alone missed PhysX convex-shape contact while a carry
    pushed a plate 14 cm. Use the existing measured contact counter (force
    above 0.01 N), with the same physics clock. Interactions belong to Anchors.
    """
    # A supporting hand exposes its manipulated object as 'tool', but the
    # pair sensor and collision counters belong to the shared physical world.
    # Keep native pair diagnostics on that owner for either hand's optimizer.
    owner=getattr(scene,'scene',scene)
    if owner.tool_target_contact_steps>contact_steps_at_entry:
        force,points,normals,separations,counts,starts=owner.tool_target_contacts.get_contact_force_data(dt=owner.dt)
        active=[np.arange(int(s),int(s)+int(c)) for c,s in zip(np.asarray(counts).ravel(),np.asarray(starts).ravel()) if c]
        indices=np.concatenate(active) if active else np.array([],dtype=int)
        owner.relaxed_contact_failure=dict(
            forces_n=np.asarray(force)[indices].reshape(-1).tolist(),
            points_world=np.asarray(points)[indices].tolist(),
            normals_world=np.asarray(normals)[indices].tolist(),
            native_separations_m=np.asarray(separations)[indices].reshape(-1).tolist(),
            evidence='native PhysX contact points; positive separation may produce contact before mesh overlap')
        raise RuntimeError('Actual RELAXED native tool-target contact')


def core_pose_errors(state, expected):
    from multipath.execution.mounted_scene import pose_matrix
    actual=pose_matrix(state['objects']['tool']['pose'])
    return {'position_m':float(np.linalg.norm(actual[:3,3]-expected[:3,3])),
            'rotation_rad':float(Rotation.from_matrix(expected[:3,:3].T@actual[:3,:3]).magnitude())}


def object_tracking_score(errors):
    """Appendix C.3/C.4 with one weighted reward/termination boundary.

    Shared intercepts remain the project's 5 cm / 30 degrees, not paper
    numerical values. Combined translation and rotation error uses C.4.
    """
    distance=float(np.hypot(errors['position_m']/OBJECT_POSITION_TOLERANCE_M,
                            errors['rotation_rad']/OBJECT_ROTATION_TOLERANCE_RAD))
    feasible=distance<=1.
    return {'weighted_error':distance, 'reward':1.-distance, 'feasible':bool(feasible)}


def hand_tool_contact(paths, forces, threshold=.01):
    """Appendix C.7 contact proxy; simultaneous thumb and another finger.

    This is contact evidence, not force closure or load-bearing proof.
    Paths must use the contact view's actual row order.
    """
    forces=np.asarray(forces).reshape(-1,3)
    if len(paths)!=len(forces) or not np.isfinite(forces).all():
        raise RuntimeError('Invalid hand contact force/path mapping')
    magnitudes=np.linalg.norm(forces,axis=1)
    touching=magnitudes>threshold
    names=[p.rsplit('/',1)[-1].lower() for p in paths]
    thumb=any(t and 'thumb' in n for n,t in zip(names,touching))
    other=any(t and any(f in n for f in ('index','mid','ring','pinky','little')) for n,t in zip(names,touching))
    return {'mode':'grasped' if thumb and other else 'free',
            'thumb_contact':bool(thumb),'other_finger_contact':bool(other),
            'opposition_contact':bool(thumb and other),
            'touching_links':int(touching.sum()),'link_paths':list(paths),
            'force_norms_n':magnitudes.tolist(),'force_vectors_n':forces.tolist(),
            'evidence':'PhysX thumb AND non-thumb finger/tool contact; retention checked separately'}
