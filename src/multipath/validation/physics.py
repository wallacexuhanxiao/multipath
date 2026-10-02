import numpy as np
from scipy.spatial.transform import Rotation


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
