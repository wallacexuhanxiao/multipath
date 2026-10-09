"""SPIDER XHand finger order used by EgoEngine qpos[..., 7:]."""

RIGHT_XHAND_FINGER_JOINT_NAMES = (
    "right_hand_thumb_bend_joint", "right_hand_thumb_rota_joint1",
    "right_hand_thumb_rota_joint2", "right_hand_index_bend_joint",
    "right_hand_index_joint1", "right_hand_index_joint2",
    "right_hand_mid_joint1", "right_hand_mid_joint2",
    "right_hand_ring_joint1", "right_hand_ring_joint2",
    "right_hand_pinky_joint1", "right_hand_pinky_joint2",
)

# SPIDER joint coordinates: positive thumb opposition/finger flexion closes
# the grasp; index_bend is lateral abduction, not a closing coordinate. These
# are embodiment semantics, independent of demonstrations and task windows.
RIGHT_XHAND_CLOSING_DIRECTION = (1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1)


def finger_joint_names(side='right'):
    if side not in ('left', 'right'):
        raise ValueError('XHand side must be left or right')
    return tuple(name.replace('right_hand_', side+'_hand_', 1)
                 for name in RIGHT_XHAND_FINGER_JOINT_NAMES)
