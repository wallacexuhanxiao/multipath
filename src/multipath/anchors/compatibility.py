"""Canonical boundary comparison includes dynamics, objects and contact."""
import numpy as np
from scipy.spatial.transform import Rotation


def boundary_errors(actual, expected):
    errors = {}
    floating=(expected.get('embodiment','').startswith('floating_xhand')
              or ('wrist_xyz_wxyz' in expected and 'arm_q' not in expected))
    errors['embodiment_match']=actual.get('embodiment')==expected.get('embodiment')
    if floating:
        a=np.asarray(actual['wrist_xyz_wxyz']);b=np.asarray(expected['wrist_xyz_wxyz'])
        errors['wrist_position_m']=float(np.linalg.norm(a[:3]-b[:3]))
        errors['wrist_rotation_rad']=float((Rotation.from_quat(a[[4,5,6,3]])*Rotation.from_quat(b[[4,5,6,3]]).inv()).magnitude())
        errors['wrist_velocity']=float(np.max(np.abs(np.asarray(actual['wrist_twist'])-expected['wrist_twist'])))
    for key in (('hand_q','hand_dq') if floating else ('arm_q','hand_q','arm_dq','hand_dq')):
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
    # The top-level state is the primary hand. A joint boundary also needs
    # the other hand's dynamics/contact, not just the shared object poses.
    for side,target in expected.get('hands',{}).items():
        if side=='right':continue
        measured=actual.get('hands',{}).get(side)
        if measured is None:
            errors[side+'_contact_match']=False
            continue
        a=np.asarray(measured['wrist_xyz_wxyz']);b=np.asarray(target['wrist_xyz_wxyz'])
        errors[side+'_wrist_position_m']=float(np.linalg.norm(a[:3]-b[:3]))
        errors[side+'_wrist_rotation_rad']=float((Rotation.from_quat(a[[4,5,6,3]])*
                                                Rotation.from_quat(b[[4,5,6,3]]).inv()).magnitude())
        errors[side+'_wrist_velocity']=float(np.max(np.abs(np.asarray(measured['wrist_twist'])-target['wrist_twist'])))
        for key in ('hand_q','hand_dq'):
            errors[side+'_'+key]=float(np.max(np.abs(np.asarray(measured[key])-target[key])))
        errors[side+'_contact_match']=measured['contact']['mode']==target['contact']['mode']
        if target['contact']['mode']=='grasped':
            errors[side+'_relative_transform']=float(np.max(np.abs(np.asarray(measured['hand_object_transform'])-target['hand_object_transform'])))
    return errors


def compatible(errors):
    return bool(errors['contact_match'] and errors['scene_match'] and
                all(v for k,v in errors.items() if k.endswith('_contact_match')) and
                all(v<.10 for k,v in errors.items() if k.endswith('_hand_q')) and
                all(v<.5 for k,v in errors.items() if k.endswith('_hand_dq')) and
                all(v<.05 for k,v in errors.items() if k.endswith('_relative_transform')) and
                errors.get('embodiment_match',True) and errors.get('arm_q',0.) < .08 and errors['hand_q'] < .10 and
                errors.get('arm_dq',0.) < .3 and errors['hand_dq'] < .5 and
                (errors.get('relative_transform_required',True) is False or errors['relative_transform'] < .05) and
                all(v < (.05 if 'position' in k else .35 if 'rotation' in k else .15)
                    for k,v in errors.items() if k.endswith(('_position_m','_rotation_rad','_velocity'))))
