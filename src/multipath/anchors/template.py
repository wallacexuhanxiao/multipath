"""Reuse a physically valid local interaction as a new-source proposal only."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from multipath.anchors.schema import InteractionAnchor
from multipath.anchors.artifact import load_validated_anchor
from multipath.execution.mounted_scene import pose_matrix
from multipath.planning.timing import physics_ticks_per_frame


def sample_pose(matrices,coordinates):
    matrices=np.asarray(matrices);grid=np.arange(len(matrices))
    x=np.clip(coordinates,0,len(matrices)-1)
    result=np.repeat(np.eye(4)[None],len(x),axis=0)
    result[:,:3,3]=np.stack([np.interp(x,grid,matrices[:,j,3]) for j in range(3)],axis=1)
    result[:,:3,:3]=Slerp(grid,Rotation.from_matrix(matrices[:,:3,:3]))(x).as_matrix()
    return result


def local_correspondence(old_data,new_data,old_anchor,new_anchor,free_start_max=0,grasped_ends=None):
    """Monotone local human/tool correspondence inside the supplied CORE.

    This retimes an initialization, not the oracle windows or source clock.
    Equal source trajectories give the identity mapping. Object motion and
    hand approach distinguish acquisition from transport within one Anchor;
    a uniform duration warp can otherwise close the fingers after lift begins.
    """
    from multipath.validation.physics import OBJECT_POSITION_TOLERANCE_M,OBJECT_ROTATION_TOLERANCE_RAD
    def descriptor(data,anchor):
        frames=np.arange(anchor.core_start,anchor.core_end+1)-anchor.synth_start
        frames=np.clip(frames,0,len(data['tool_pose'])-1)
        tool=data['tool_pose'][frames];hand=data['wrist_hand_q'][frames]
        relative=np.linalg.inv(tool[0])@tool
        approach=np.linalg.norm(hand[:,:3]-tool[:,:3,3],axis=1)
        return np.c_[relative[:,:3,3]/OBJECT_POSITION_TOLERANCE_M,
            Rotation.from_matrix(relative[:,:3,:3]).as_rotvec()/OBJECT_ROTATION_TOLERANCE_RAD,
            approach/OBJECT_POSITION_TOLERANCE_M,hand[:,7:]]
    old=descriptor(old_data,old_anchor);new=descriptor(new_data,new_anchor)
    cost=np.mean((new[:,None,:]-old[None,:,:])**2,axis=2)
    values=np.full((len(new)+1,len(old)+1),np.inf)
    parent=np.zeros(cost.shape,dtype=np.int8)
    free_start_max=min(free_start_max,len(old)-1)
    values[1,1:free_start_max+2]=cost[0,:free_start_max+1]
    for i in range(len(new)):
        for j in range(len(old)):
            if i==0:continue
            options=(values[i,j],values[i,j+1],values[i+1,j])
            step=int(np.argmin(options));parent[i,j]=step
            values[i+1,j+1]=cost[i,j]+options[step]
    ends=[len(old)-1] if grasped_ends is None else grasped_ends
    end=min(ends,key=lambda j:values[len(new),j+1])
    pairs=[];i,j=len(new)-1,end
    while i>=0 and j>=0:
        pairs.append((i,j));step=parent[i,j]
        if i==0:break
        if step==0:i-=1;j-=1
        elif step==1:i-=1
        else:j-=1
    mapping=np.array([np.mean([j for i,j in pairs if i==k]) for k in range(len(new))])
    mapping[-1]=end
    return mapping


def acquisition_proposals(artifacts,proposals,manifest,schedule,local,dt,planning=None):
    """Object-frame transfer, fixed new clock; never copy entry validity flags.

    The library contains local Anchor artifacts, not a full robot reference.
    Tool geometry and contact transition must match. New boundary states must
    be reached by cuRobo and every adapted command must pass fresh physics.
    """
    old_manifest=json.loads((Path(proposals)/'manifest.json').read_text());result={}
    if old_manifest['initial_objects']['tool']['id']!=manifest['initial_objects']['tool']['id']:
        return result
    for path in sorted(Path(artifacts).glob('*/report.json')):
        report=json.loads(path.read_text())
        if (report['entry_contact'],report['exit_contact'])!=('free','grasped'):continue
        old=InteractionAnchor(**{key:report[key] for key in InteractionAnchor.__dataclass_fields__})
        artifact=load_validated_anchor(Path(artifacts),old,provenance=old_manifest['provenance'],dt=dt)
        data=dict(np.load(Path(proposals)/old_manifest['anchors'][old.id],allow_pickle=False))
        trace_path=path.parent/'local_actual_trace.json'
        if not trace_path.exists():trace_path=Path(artifact['entry']['entry_provenance']['trace'])
        trace=json.loads(trace_path.read_text()) if trace_path.exists() else []
        physical={round(s['time_s']*old_manifest['fps']):s for s in trace
            if old.core_start<=s['time_s']*old_manifest['fps']<=old.core_end}
        physical[old.core_start]=artifact['entry'];physical[old.core_end]=artifact['exit']
        if set(physical)!=set(range(old.core_start,old.core_end+1)):
            raise ValueError('Local initialization requires measured CORE states, not just drive targets')
        measured=[physical[f] for f in range(old.core_start,old.core_end+1)]
        from multipath.planning.bridge import coordinate_velocity
        grasped_ends=sorted({int(round(s['time_s']*old_manifest['fps']-old.core_start)) for s in measured
            if s['contact']['mode']=='grasped' and (planning is None or np.all(
                np.abs(coordinate_velocity(s['wrist_xyz_wxyz'],s['wrist_twist']))
                <=np.asarray(planning['velocity_limits'])*1.0001))})
        # An interaction-valid frame can still be too fast to enter a bridge.
        # Reusing that endpoint already caused a real fixed-time carry failure.
        # It is an initialization filter; fresh physical handoff is mandatory.
        if not grasped_ends:continue
        old_fps=old_manifest['fps'];old_tpf=physics_ticks_per_frame(old_fps,dt)
        for anchor in schedule.anchors.values():
            if (anchor.entry_contact,anchor.exit_contact)!=('free','grasped') or anchor.id in result:continue
            tpf=physics_ticks_per_frame(schedule.fps,dt);ticks=(anchor.core_end-anchor.core_start)*tpf
            # Endpoints and source durations are those of the new oracle.
            # The artifact entry is a validated handoff. A later frame with
            # no opposition contact is not necessarily a statically reachable
            # pregrasp: fingers may already be moving around the bowl. Keep
            # this boundary fixed and align the interaction inside CORE.
            correspondence=local_correspondence(data,local[anchor.id],old,anchor,0,grasped_ends)
            from scipy.interpolate import PchipInterpolator
            source_sample=np.arange(ticks+1)/tpf
            old_sample=PchipInterpolator(np.arange(len(correspondence)),correspondence)(source_sample)
            phase=old_sample[1:]/(old.core_end-old.core_start)
            new_frame=anchor.core_start-anchor.synth_start+np.arange(1,ticks+1)/tpf
            old_frame=old.core_start-old.synth_start+old_sample[1:]
            old_tool=sample_pose(data['tool_pose'],old_frame)
            new_tool=sample_pose(local[anchor.id]['tool_pose'],new_frame)
            old_c=artifact['commands'];command_grid=np.arange(len(old_c))
            old_tick=phase*len(old_c)-1
            # A drive target contains the contact/load response learned by
            # local synthesis. A measured body pose is not its replacement
            # under the same PD controller. Transfer commands in the source
            # object frame; measured states supply reachability evidence only.
            drive_poses=np.asarray([pose_matrix(artifact['entry']['command_wrist']),
                *[pose_matrix(c[:7]) for c in old_c]])
            poses=sample_pose(drive_poses,old_sample[1:]*old_tpf)
            warped=new_tool@np.linalg.inv(old_tool)@poses
            commands=np.c_[warped[:,:3,3],Rotation.from_matrix(warped[:,:3,:3]).as_quat()[:,[3,0,1,2]],
                np.stack([np.interp(old_tick,command_grid,old_c[:,j]) for j in range(7,19)],axis=1),
                np.stack([np.interp(old_tick,command_grid,old_c[:,j]) for j in range(19,25)],axis=1)]
            # Preserve optimized drive offsets and nominal feedforward. Do
            # not differentiate pose residuals into new velocity impulses.
            scale=np.diff(old_sample)/(old_fps*dt)
            previous_old=sample_pose(data['tool_pose'],old_frame-np.diff(old_sample))
            previous_new=sample_pose(local[anchor.id]['tool_pose'],new_frame-1/tpf)
            old_v=(old_tool[:,:3,3]-previous_old[:,:3,3])/dt
            new_v=(new_tool[:,:3,3]-previous_new[:,:3,3])/dt
            old_w=Rotation.from_matrix(old_tool[:,:3,:3]@previous_old[:,:3,:3].transpose(0,2,1)).as_rotvec()/dt
            new_w=Rotation.from_matrix(new_tool[:,:3,:3]@previous_new[:,:3,:3].transpose(0,2,1)).as_rotvec()/dt
            orientation=new_tool[:,:3,:3]@old_tool[:,:3,:3].transpose(0,2,1)
            old_offset=poses[:,:3,3]-old_tool[:,:3,3];new_offset=warped[:,:3,3]-new_tool[:,:3,3]
            commands[:,19:22]=new_v+np.cross(new_w,new_offset)+np.einsum('nij,nj->ni',orientation,
                scale[:,None]*commands[:,19:22]-old_v-np.cross(old_w,old_offset))
            commands[:,22:25]=new_w+np.einsum('nij,nj->ni',orientation,scale[:,None]*commands[:,22:25]-old_w)
            entry_phase=float(correspondence[0]);entry_tick=entry_phase*old_tpf-1
            entry_drive=(np.r_[artifact['entry']['command_wrist'],artifact['entry']['command_hand_q'],artifact['entry']['command_twist']]
                if entry_phase==0 else np.array([np.interp(entry_tick,command_grid,old_c[:,j]) for j in range(25)]))
            old_boundary=sample_pose(data['tool_pose'],[old.core_start-old.synth_start+entry_phase])[0]
            new_boundary=local[anchor.id]['tool_pose'][anchor.core_start-anchor.synth_start]
            boundary_transform=new_boundary@np.linalg.inv(old_boundary)
            entry_pose=boundary_transform@pose_matrix(entry_drive[:7])
            old_entry=artifact['entry']
            expected_pose=boundary_transform@pose_matrix(old_entry['wrist_xyz_wxyz'])
            entry=dict(wrist_xyz_wxyz=np.r_[entry_pose[:3,3],Rotation.from_matrix(entry_pose[:3,:3]).as_quat()[[3,0,1,2]]].tolist(),
                wrist_twist=np.zeros(6).tolist(),hand_q=old_entry['hand_q'],hand_dq=np.zeros(12).tolist(),
                command_hand_q=entry_drive[7:19].tolist())
            result[anchor.id]=dict(entry=entry,commands=commands,
                expected_wrist_state=expected_pose.tolist(),previous_drive_wrist=entry['wrist_xyz_wxyz'],
                previous_drive_twist=entry['wrist_twist'],previous_drive_hand=entry['command_hand_q'],provenance=dict(
                artifact=str(path.parent),source_provenance=report['source_provenance'],
                method='source_object_frame_validated_local_drive_initialization',physical_acceptance=False,
                local_source_correspondence=correspondence.tolist(),oracle_schedule_changed=False,
                endpoint_filter='native grasp and configured planning velocity budget' if planning else 'native grasp',
                free_pregrasp_phase=entry_phase,free_pregrasp_physical_evidence=str(trace_path),
                new_source_budget_s=(anchor.core_end-anchor.core_start)/schedule.fps))
    return result
