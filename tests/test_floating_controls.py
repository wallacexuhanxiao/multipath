"""Small checks for the new wrist coordinate/interface failure modes."""
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from multipath.planning.floating import coordinates_pose,pose_coordinates
from multipath.anchors.floating import controls_from_poses,local_controls


def test_virtual_wrist_coordinates_roundtrip():
    q=np.array([.3,-.2,.5,.7,-.4,.2]);pose=coordinates_pose(q)
    np.testing.assert_allclose(pose_coordinates(pose,q),q,atol=1e-10)
    opposite=pose.copy();opposite[3:]*=-1
    np.testing.assert_allclose(pose_coordinates(opposite,q),q,atol=1e-10)


def test_world_angular_feedforward_and_fixed_clock():
    dt=1/120;poses=[]
    for k in range(5):
        r=Rotation.from_rotvec([0,0,k*dt])
        poses.append(np.r_[[k*dt,0,0],r.as_quat()[[3,0,1,2]]])
    controls=controls_from_poses(poses,np.zeros((5,12)),dt)
    assert controls.shape==(4,25)
    np.testing.assert_allclose(controls[:,19:25],np.tile([1,0,0,0,0,1],(4,1)),atol=1e-10)


def test_padding_correction_ends_before_core():
    anchor=SimpleNamespace(synth_start=0,core_start=2,core_end=4,synth_end=5,entry_contact="free",exit_contact="grasped")
    scene=SimpleNamespace(dt=1/120,manifest={'fps':30},hand_limits=np.tile([-2,2],(12,1)),hand_velocity=np.full(12,10.))
    ref=np.zeros((5,19));ref[:,3]=1.;ref[:,0]=np.arange(5)*.01
    start={'wrist_xyz_wxyz':[.2,0,0,1,0,0,0],'hand_q':np.zeros(12)}
    c=local_controls(scene,anchor,{'wrist_hand_q':ref},start)
    assert c.shape==(20,25)
    np.testing.assert_allclose(c[7,:7],ref[2,:7],atol=1e-10)


def test_measured_collision_does_not_add_tracking_tube_twice():
    from multipath.planning.floating import FloatingPlanner
    planner=object.__new__(FloatingPlanner)
    planner.scene=SimpleNamespace(root_motion=lambda:(np.array([0.,0.,1.06,1.,0,0,0]),np.zeros(6)))
    planner.nominal_spheres=[{'center':[0.,0.,0.],'radius':.05}]
    planner.live={'cuboid':{'support':{'pose':[0.,0.,0.,1.,0.,0.,0.],'dims':[2.,2.,2.]}}}
    assert planner.measured_valid()
    planner.scene.root_motion=lambda:(np.array([0.,0.,1.04,1.,0,0,0]),np.zeros(6))
    assert not planner.measured_valid()


def test_sampled_pose_residual_keeps_nominal_velocity_feedforward():
    from multipath.anchors.floating_search import apply_residual
    scene=SimpleNamespace(dt=1/120,hand_limits=np.tile([-2.,2.],(12,1)),hand_velocity=np.full(12,10.))
    baseline=np.zeros((8,25));baseline[:,3]=1.;baseline[:,19]=.15
    start={'wrist_xyz_wxyz':[0.,0.,0.,1.,0.,0.,0.],'hand_q':np.zeros(12)}
    residual=np.zeros((2,18));residual[:,0]=[.02,-.02]
    controls=apply_residual(scene,baseline,start,residual,4,np.ones(9))
    np.testing.assert_allclose(controls[:,19:25],baseline[:,19:25])
    assert controls[3,0]>.019 and controls[-1,0]<-.019


def test_measured_grasp_proposal_does_not_jump_the_existing_drive_target():
    from multipath.anchors.floating import object_relative_command,object_relative_handoff
    anchor=SimpleNamespace(synth_start=0,core_end=10,synth_end=12)
    tool=np.tile(np.eye(4),(13,1,1));tool[:,0,3]=np.arange(13)*.002
    tool[:,:3,:3]=Rotation.from_rotvec(np.c_[np.zeros((13,2)),np.arange(13)*.04]).as_matrix()
    pose=np.r_[[.024,0,0],Rotation.from_rotvec([0,0,.18]).as_quat()[[3,0,1,2]]]
    scene=SimpleNamespace(dt=1/120,manifest={'fps':30},command_pose=pose,hand_command=np.zeros(12),command_twist=np.zeros(6),
        cfg={'planning':{'velocity_limits':[1,1,1,3,3,3]}})
    previous=np.r_[pose,np.zeros(18)];data={'tool_pose':tool};held=np.eye(4)
    handoff=object_relative_handoff(scene,anchor,data,2.,held)
    entry=object_relative_command(scene,anchor,data,2.,previous,held,handoff)
    np.testing.assert_allclose(entry[:7],pose,atol=1e-10)
    np.testing.assert_allclose(object_relative_command(scene,anchor,data,2.25,previous,held,handoff)[19:25],scene.command_twist,atol=1e-10)
    first=object_relative_command(scene,anchor,data,2.25,previous,held,handoff)
    assert np.linalg.norm(first[:3]-pose[:3])<.005
    end=2+handoff['duration_s']*30
    np.testing.assert_allclose(object_relative_command(scene,anchor,data,end,previous,held,handoff)[:7],
                               object_relative_command(scene,anchor,data,end,previous,held)[:7],atol=1e-10)


def test_padded_takeover_matches_measured_velocity_without_changing_core():
    anchor=SimpleNamespace(synth_start=0,core_start=2,core_end=4,synth_end=5,entry_contact="free",exit_contact="grasped")
    scene=SimpleNamespace(dt=1/120,manifest={'fps':30},hand_limits=np.tile([-2,2],(12,1)),hand_velocity=np.full(12,10.))
    ref=np.zeros((5,19));ref[:,3]=1.;ref[:,0]=np.arange(5)*.01
    start={'wrist_xyz_wxyz':[.02,0,0,1,0,0,0],'wrist_twist':[.1,.02,0,0,.1,.02],
           'hand_q':np.full(12,.1),'hand_dq':np.full(12,.2)}
    c=local_controls(scene,anchor,{'wrist_hand_q':ref},start)
    np.testing.assert_allclose(c[0,19:25],start['wrist_twist'],atol=1e-10)
    np.testing.assert_allclose((c[0,7:19]-start['hand_q'])/scene.dt,start['hand_dq'],atol=1e-10)
    np.testing.assert_allclose(c[7,:19],ref[2],atol=1e-10)


def test_grasp_preload_is_not_counted_as_a_new_command_jump():
    from multipath.anchors.floating import validate_commands
    from multipath.anchors.floating_search import apply_residual
    scene=SimpleNamespace(dt=.01,hand_limits=np.tile([-2.,2.],(12,1)),hand_velocity=np.ones(12))
    start={'wrist_xyz_wxyz':[0.,0.,0.,1.,0.,0.,0.],'hand_q':np.zeros(12),
           'command_hand_q':np.full(12,.1),'contact':{'mode':'grasped'}}
    baseline=np.zeros((4,25));baseline[:,3]=1.;baseline[:,7:19]=.1
    assert validate_commands(scene,baseline,start)['feasible']
    residual=np.zeros((1,18));residual[0,0]=.001
    command=apply_residual(scene,baseline,start,residual,4,np.ones(5))
    np.testing.assert_allclose(command[:,7:19],baseline[:,7:19])
    anchor=SimpleNamespace(synth_start=0,core_start=2,core_end=4,synth_end=5,entry_contact="free",exit_contact="grasped")
    scene.dt=1/120;scene.manifest={'fps':30}
    ref=np.zeros((5,19));ref[:,3]=1.;ref[:,7:]=.1
    controls=local_controls(scene,anchor,{'wrist_hand_q':ref},start)
    assert validate_commands(scene,controls,start)['feasible']
    np.testing.assert_allclose(controls[0,7:19],start['command_hand_q'])
    # A transient absence of opposition does not erase the previous drive.
    start['contact']['mode']='free'
    assert validate_commands(scene,baseline,start)['feasible']
    anchor.entry_contact=anchor.exit_contact='grasped'
    start['hand_object_transform']=np.eye(4)
    data={'wrist_hand_q':ref,'tool_pose':np.tile(np.eye(4),(len(ref),1,1))}
    ref[:,7:]=.4
    controls=local_controls(scene,anchor,data,start)
    assert np.all(controls[:,7:19]>=np.asarray(start['command_hand_q'])-1e-10)
    assert controls[-1,7]>.1
    ref[:,7:]=0.
    controls=local_controls(scene,anchor,data,start)
    np.testing.assert_allclose(controls[:,7:19],np.tile(start['command_hand_q'],(len(controls),1)))


def test_carry_anchor_uses_acquired_grasp_and_finishes_context_at_core():
    from multipath.execution.mounted_scene import pose_matrix
    anchor=SimpleNamespace(synth_start=0,core_start=3,core_end=5,synth_end=6,
                           entry_contact='grasped',exit_contact='grasped')
    scene=SimpleNamespace(dt=1/120,manifest={'fps':30},hand_limits=np.tile([-2,2],(12,1)),hand_velocity=np.full(12,10.))
    ref=np.zeros((6,19));ref[:,3]=1.
    objects=np.tile(np.eye(4),(6,1,1));objects[:,0,3]=np.arange(6)*.01
    objects[:,:3,:3]=Rotation.from_rotvec(np.c_[np.zeros((6,2)),np.arange(6)*.04]).as_matrix()
    held=np.eye(4);held[:3,3]=[.1,.02,0];held[:3,:3]=Rotation.from_rotvec([.1,.2,-.3]).as_matrix()
    start={'wrist_xyz_wxyz':[.02,0,0,1,0,0,0],'wrist_twist':[.1,.02,0,0,.1,.02],
           'hand_q':np.zeros(12),'hand_dq':np.zeros(12),'hand_object_transform':held}
    controls=local_controls(scene,anchor,{'wrist_hand_q':ref,'tool_pose':objects},start)
    np.testing.assert_allclose(controls[0,19:25],start['wrist_twist'],atol=1e-10)
    # Source boundary is reached without assuming the human/robot grasp matches.
    np.testing.assert_allclose(pose_matrix(controls[11,:7])@held,objects[3],atol=1e-10)
    np.testing.assert_allclose(pose_matrix(controls[19,:7])@held,objects[5],atol=1e-10)


def test_relaxed_pregrasp_boundary_uses_fixed_time_and_rejects_short_budget():
    from multipath.planning.floating import relaxed_hand_path
    import pytest
    scene=SimpleNamespace(dt=.01,hand_limits=np.tile([-2.,2.],(12,1)),hand_velocity=np.ones(12))
    start={'hand_q':np.zeros(12),'hand_dq':np.zeros(12),'contact':{'mode':'free'}}
    goal={'hand_q':np.full(12,.35),'hand_dq':np.zeros(12)}
    path=relaxed_hand_path(scene,start,goal,100)
    assert path.shape==(101,12)
    np.testing.assert_allclose(path[[0,-1]],np.stack((start['hand_q'],goal['hand_q'])))
    with pytest.raises(RuntimeError,match='source-time'):
        relaxed_hand_path(scene,start,goal,20)
