import numpy as np
from .physics import core_pose_errors
from multipath.execution.mounted_scene import pose_matrix


def evaluate_task(state,manifest,contract,anchor_reports):
    err=core_pose_errors(state,pose_matrix(manifest['final_objects']['tool']))
    speed=float(np.linalg.norm(state['objects']['tool']['velocity'][:3]))
    passed=bool(anchor_reports and all(a['local_rollout_success'] and a['handoff_verified'] for a in anchor_reports)
                and err['position_m']<contract['position_tolerance_m']
                and err['rotation_rad']<contract['rotation_tolerance_rad']
                and speed<contract['stable_speed_m_s'] and state['contact']['mode']=='free')
    return dict(success=passed,final_errors=err,final_speed_m_s=speed,released=state['contact']['mode']=='free')
