"""Check the handoff after physical execution; never assign the goal state."""
import numpy as np
from multipath.anchors.compatibility import boundary_errors, compatible


def check_transition(scene,planned_q,expected_contact,expected_object=None,canonical=None):
    s=scene.state()
    report={'arm_q_error_rad':float(np.max(np.abs(np.array(s['arm_q'])-planned_q))),
            'actual_arm_dq':s['arm_dq'],'actual_hand_q':s['hand_q'],'actual_hand_dq':s['hand_dq'],
            'actual_objects':s['objects'],'actual_contact':s['contact'],
            'actual_hand_object_transform':s['hand_object_transform'], 'scene_id':s['scene_id']}
    good=report['arm_q_error_rad']<.08 and s['contact']['mode']==expected_contact
    if expected_object is not None:
        from multipath.validation.physics import core_pose_errors
        report['object_boundary_error']=core_pose_errors(s,expected_object)
        good=good and report['object_boundary_error']['position_m']<.05 and report['object_boundary_error']['rotation_rad']<.35
    if canonical is not None:
        report['canonical_errors']=boundary_errors(s,canonical)
        good=good and compatible(report['canonical_errors'])
    report['success']=bool(good)
    return report
