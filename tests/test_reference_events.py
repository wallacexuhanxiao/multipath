import numpy as np
from scipy.spatial.transform import Rotation

from multipath.reference_events import compile_reference_schedule


def reference_and_vlm(frames=150, scale=1., rotation=None, shift=None):
    t=np.linspace(0,1,frames)
    movement=np.clip((t-.25)/.15,0,1)*np.clip((.8-t)/.15,0,1)
    object_xyz=np.c_[.1*movement,np.zeros(frames),.15*movement]
    angles=.9*movement
    object_rot=Rotation.from_euler('y',angles).as_matrix()
    distance=.3-.15*np.clip((t-.1)/.15,0,1)+.15*np.clip((t-.8)/.1,0,1)
    wrist=object_xyz+np.c_[distance,np.zeros(frames),np.zeros(frames)]
    rotation=np.eye(3) if rotation is None else rotation
    shift=np.zeros(3) if shift is None else shift
    poses=np.tile(np.eye(4),(frames,1,1))
    poses[:,:3,:3]=rotation@object_rot
    poses[:,:3,3]=scale*object_xyz@rotation.T+shift
    q=np.zeros((frames,19));q[:,:3]=scale*wrist@rotation.T+shift;q[:,3]=1
    vlm={'instruction':'move the held object through the interaction and release',
         'num_frames':frames,'fps':frames/5,
         'schedule':[{'start':0,'end':frames,'mode':'RELAXED'}]}
    return {'qpos_right_actual':q,'object_pose_gt_tool':poses},vlm


def test_reference_events_follow_clock_scale_and_coordinate_changes():
    reference,vlm=reference_and_vlm()
    first=compile_reference_schedule(reference,vlm,vlm['fps'])
    rotation=Rotation.from_euler('xyz',[.4,-.3,.2]).as_matrix()
    other,other_vlm=reference_and_vlm(scale=2.,rotation=rotation,shift=[3.,-2.,1.])
    second=compile_reference_schedule(other,other_vlm,other_vlm['fps'])
    for event in ('grasp_contact_frame','grasp_closed_frame','release_frame'):
        assert first[event]==second[event]
    assert first['schedule']==second['schedule']
    doubled,vlm2=reference_and_vlm(frames=300)
    third=compile_reference_schedule(doubled,vlm2,vlm2['fps'])
    for event in ('grasp_contact_frame','grasp_closed_frame','release_frame'):
        assert abs(first[event]/vlm['fps']-third[event]/vlm2['fps'])<.05
    assert first['reference_driven'] and first['event_evidence']['physical_validation']=='pending'


def test_upstream_bc_is_preserved_and_acquisition_checkpoint_is_explicit():
    reference,vlm=reference_and_vlm()
    vlm['schedule']=[{'start':0,'end':10,'mode':'BC'},
                     {'start':10,'end':150,'mode':'RELAXED'}]
    generated=compile_reference_schedule(reference,vlm,vlm['fps'])
    assert generated['schedule'][0]=={'start':0,'end':10,'mode':'BC'}
    assert any(w['end']==generated['acquisition_end_frame'] and w['mode']=='BC'
               for w in generated['schedule'])
