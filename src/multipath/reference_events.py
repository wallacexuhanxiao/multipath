"""Compile source-clock events from trajectories and upstream VLM BC labels.

No action names, dataset IDs, object IDs, or source frame numbers select code
paths. This compiler supports one moving rigid object held by one hand.
"""
import numpy as np
from scipy.ndimage import binary_closing
from scipy.spatial.transform import Rotation


def sustained_first(mask, count):
    indices=np.flatnonzero(np.convolve(np.asarray(mask,dtype=int),np.ones(count,dtype=int),'valid')>=count)
    if not len(indices):raise ValueError('Reference does not contain a sustained event')
    return int(indices[0])


def compile_reference_schedule(reference, vlm, fps):
    wrist=np.asarray(reference['qpos_right_actual'],dtype=float)
    poses=np.asarray(reference['object_pose_gt_tool'],dtype=float)
    n=len(wrist)
    if len(poses)!=n or n<10 or fps<=0:
        raise ValueError('Reference and source video must have matching nonempty clocks')
    if int(vlm['num_frames'])!=n or abs(float(vlm['fps'])-fps)>.05:
        raise ValueError('VLM, video, and reference clocks disagree')
    position=poses[:,:3,3]
    rotation=Rotation.from_matrix(poses[:,:3,:3])
    displacement=np.linalg.norm(position-position[0],axis=1)
    angular=(rotation[0].inv()*rotation).magnitude()
    translation_span=float(displacement.max())
    rotation_span=float(angular.max())
    if translation_span<.01:
        raise ValueError('Cannot infer a held-object acquisition from a stationary reference')
    persistence=max(2,int(round(.1*fps)))
    contact=sustained_first(displacement>.05*translation_span,persistence)
    closed=max(contact+1,sustained_first(displacement>.15*translation_span,persistence))
    acquisition_end=closed+1
    distance=np.linalg.norm(wrist[:,:3]-position,axis=1)
    middle=(displacement>.5*translation_span)
    grasp_distance=float(np.median(distance[middle]))
    excursion=max(float(np.percentile(distance,90)-grasp_distance),.02)
    release_mask=distance>grasp_distance+.25*excursion
    release_mask[:int(displacement.argmax())+1]=False
    release=sustained_first(release_mask,persistence)
    if not contact<closed<release<n-1:
        raise ValueError('Reference does not support ordered acquisition and release events')
    bc=np.zeros(n,dtype=bool)
    for item in vlm['schedule']:
        if item['mode']=='BC':bc[int(item['start']):int(item['end'])]=True
    # A large orientation excursion is an interaction constraint, including
    # its return; allowing free planning here can erase the demonstrated act.
    orientation_critical=angular>max(.12,.15*rotation_span)
    orientation_critical=binary_closing(orientation_critical,iterations=persistence)
    bc|=orientation_critical
    lead=max(1,int(round(.35*fps)))
    release_pad=max(1,int(round(.2*fps)))
    bc[max(0,contact-lead):acquisition_end]=True
    bc[max(closed,release-release_pad):min(n,release+release_pad)]=True
    # Preserve a separate BC handoff for measuring physical acquisition even
    # when interaction BC immediately follows it.
    cuts=sorted(set([0,n,acquisition_end,*list(np.flatnonzero(bc[1:]!=bc[:-1])+1)]))
    windows=[{'start':int(a),'end':int(b),'mode':'BC' if bc[a] else 'RELAXED'}
             for a,b in zip(cuts,cuts[1:])]
    acquisition_start=max(w['start'] for w in windows if w['mode']=='BC' and w['start']<acquisition_end)
    return {'instruction':vlm['instruction'],'num_frames':n,'fps':float(fps),
            'reference_driven':True,'initial_arm_pose':'automatic_source',
            'schedule':windows,
            'bc_intervals':[{'start':w['start'],'end':w['end']} for w in windows if w['mode']=='BC'],
            'grasp_contact_frame':contact,'grasp_closed_frame':closed,
            'acquisition_end_frame':acquisition_end,'release_frame':release,
            'precontact_end_frame':max(acquisition_start+1,contact),
            'event_evidence':{'translation_span_m':translation_span,'orientation_span_rad':rotation_span,
                              'held_wrist_object_distance_m':grasp_distance,
                              'event_source':'reference motion and hand withdrawal; upstream VLM BC union',
                              'physical_validation':'pending'}}
