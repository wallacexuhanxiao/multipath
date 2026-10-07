"""Interaction handoffs for diversity, followed by actual continuation.

V0's canonical-state comparison remains unchanged. Alternative paths must
retain the interaction state, not reproduce a particular contact impulse.
Their measured velocities are passed to the next planner/controller.
"""
import copy
import numpy as np
from scipy.spatial.transform import Rotation
from multipath.anchors.compatibility import boundary_errors,compatible
from multipath.validation.physics import core_pose_errors,object_tracking_score
from multipath.execution.mounted_scene import pose_matrix


def grasp_conditioned_boundary(expected,reached):
    """A planning target with the same tool pose and the actually held frame.

    This is not an acquired state or a snapshot that may be restored.
    """
    target=copy.deepcopy(expected)
    if expected['contact']['mode']=='grasped':
        matrix=pose_matrix(expected['objects']['tool']['pose'])@np.linalg.inv(reached['hand_object_transform'])
        target['wrist_xyz_wxyz']=np.r_[matrix[:3,3],Rotation.from_matrix(matrix[:3,:3]).as_quat()[[3,0,1,2]]].tolist()
        target['hand_object_transform']=copy.deepcopy(reached['hand_object_transform'])
    target.update(physically_acquired_state=False,state_origin='interaction_boundary_planning_target')
    target.pop('entry_origin',None);target.pop('entry_provenance',None)
    return target


def check_interaction_boundary(scene,expected,contact,terminal=False):
    actual=scene.state();target=grasp_conditioned_boundary(expected,actual)
    errors=boundary_errors(actual,target)
    pose_error=core_pose_errors(actual,pose_matrix(expected['objects']['tool']['pose']))
    static=(errors['embodiment_match'] and errors['scene_match'] and
            actual['contact']['mode']==contact and errors['hand_q']<.10 and
            errors.get('wrist_position_m',0)<.05 and errors.get('wrist_rotation_rad',0)<.35)
    if terminal:
        if contact!='free':raise ValueError('Only a released terminal interval has no following interaction boundary')
        # Fingers can still converge to their unchanged open-hand drive after
        # release. No later Anchor requires a specific measured hand/wrist pose.
        # Keep physical limits/collision in execution and actual task acceptance.
        static=errors['embodiment_match'] and errors['scene_match'] and actual['contact']['mode']=='free'
    environment=all(value<(.05 if key.endswith('_position_m') else .35 if key.endswith('_rotation_rad') else .15)
        for key,value in errors.items()
        if key.endswith(('_position_m','_rotation_rad','_velocity'))
        and not key.startswith(('wrist_','tool_')))
    drive_error=float(np.max(np.abs(np.asarray(actual['command_hand_q'])-
                                  expected.get('command_hand_q',expected['hand_q']))))
    return dict(success=bool(static and environment and drive_error<.10 and object_tracking_score(pose_error)['feasible']),
                acceptance=('terminal_release_environment_and_continuous_physical_execution' if terminal else
                    'interaction_pose_contact_environment_and_continuous_physical_execution'),
                terminal_pose_tracking_diagnostic_only=bool(terminal),
                canonical_errors=errors,canonical_dynamics_match_diagnostic=compatible(errors),
                object_boundary_error=pose_error,finger_drive_error_rad=drive_error,
                actual_contact=actual['contact'],actual_wrist_twist=actual['wrist_twist'],
                actual_state_propagated=True,state_assigned=False)
