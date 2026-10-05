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
    anchor=SimpleNamespace(synth_start=0,core_start=2,core_end=4,synth_end=5)
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
