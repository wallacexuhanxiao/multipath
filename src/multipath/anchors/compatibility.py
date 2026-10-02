"""Canonical boundary comparison includes dynamics, objects and contact."""
import numpy as np
from scipy.spatial.transform import Rotation


def boundary_errors(actual, expected):
    errors = {}
    for key in ('arm_q','hand_q','arm_dq','hand_dq'):
        errors[key] = float(np.max(np.abs(np.array(actual[key])-expected[key])))
    for role, target in expected['objects'].items():
        measured = actual['objects'][role]
        errors[role+'_position_m'] = float(np.linalg.norm(np.array(measured['pose'][:3])-target['pose'][:3]))
        errors[role+'_rotation_rad'] = float((Rotation.from_quat(np.array(measured['pose'])[ [4,5,6,3] ]).inv()*
                                              Rotation.from_quat(np.array(target['pose'])[ [4,5,6,3] ])).magnitude())
        errors[role+'_velocity'] = float(np.max(np.abs(np.array(measured['velocity'])-target['velocity'])))
    errors['relative_transform'] = float(np.max(np.abs(np.array(actual['hand_object_transform'])-expected['hand_object_transform'])))
    errors['relative_transform_required'] = expected['contact']['mode']=='grasped'
    errors['contact_match'] = actual['contact']['mode'] == expected['contact']['mode']
    errors['scene_match'] = actual['scene_id'] == expected['scene_id']
    return errors


def compatible(errors):
    return bool(errors['contact_match'] and errors['scene_match'] and
                errors['arm_q'] < .08 and errors['hand_q'] < .10 and
                errors['arm_dq'] < .3 and errors['hand_dq'] < .5 and
                (errors.get('relative_transform_required',True) is False or errors['relative_transform'] < .05) and
                all(v < (.05 if 'position' in k else .35 if 'rotation' in k else .15)
                    for k,v in errors.items() if k.endswith(('_position_m','_rotation_rad','_velocity'))))
