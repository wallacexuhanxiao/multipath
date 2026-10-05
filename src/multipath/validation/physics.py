import numpy as np
from scipy.spatial.transform import Rotation

OBJECT_POSITION_TOLERANCE_M = .05
OBJECT_ROTATION_TOLERANCE_RAD = float(np.deg2rad(30.))


def object_tracking_parameters():
    return {'C':1., 'lambda_p':1/OBJECT_POSITION_TOLERANCE_M**2,
            'lambda_R':1/OBJECT_ROTATION_TOLERANCE_RAD**2,
            'position_tolerance_m':OBJECT_POSITION_TOLERANCE_M,
            'rotation_tolerance_rad':OBJECT_ROTATION_TOLERANCE_RAD,
            'acceptance':'independent_position_and_rotation_limits'}


def retention_errors(state, held_transform):
    delta=np.linalg.inv(held_transform)@np.asarray(state['hand_object_transform'])
    return {'position_m':float(np.linalg.norm(delta[:3,3])),
            'rotation_rad':float(Rotation.from_matrix(delta[:3,:3]).magnitude()),
            'contact_retained':state['contact']['mode']=='grasped'}


def check_retention(state,held_transform):
    errors=retention_errors(state,held_transform)
    if errors['position_m']>.05 or errors['rotation_rad']>.35 or not errors['contact_retained']:
        raise RuntimeError(f'Persistent grasp constraint failed: {errors}')
    return errors


def core_pose_errors(state, expected):
    from multipath.execution.mounted_scene import pose_matrix
    actual=pose_matrix(state['objects']['tool']['pose'])
    return {'position_m':float(np.linalg.norm(actual[:3,3]-expected[:3,3])),
            'rotation_rad':float(Rotation.from_matrix(expected[:3,:3].T@actual[:3,:3]).magnitude())}


def object_tracking_score(errors):
    """Weighted object reward and the project's explicit pose tolerances.

    Project axial tolerances are 5 cm / 30 degrees, shared by Replay, MPC,
    PPO and core handoff. These are project choices, not paper values.
    Reward retains the Appendix-style weighted norm. Acceptance checks each
    requested tolerance directly, so position error cannot shrink the allowed
    rotation below 30 degrees (or vice versa).
    """
    distance=float(np.hypot(errors['position_m']/OBJECT_POSITION_TOLERANCE_M,
                            errors['rotation_rad']/OBJECT_ROTATION_TOLERANCE_RAD))
    feasible=(errors['position_m']<=OBJECT_POSITION_TOLERANCE_M and
              errors['rotation_rad']<=OBJECT_ROTATION_TOLERANCE_RAD)
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
            'force_norms_n':magnitudes.tolist(),
            'evidence':'PhysX thumb AND non-thumb finger/tool contact; retention checked separately'}
