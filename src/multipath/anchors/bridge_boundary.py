"""Local Anchor residuals for a physically incompatible outgoing bridge.

The relaxed planner's source time and limits remain fixed. This module only
proposes changes to the preceding interaction; a fresh PhysX rollout decides
whether its interaction and outgoing bridge are both valid.
"""
import numpy as np
from scipy.optimize import linprog
from scipy.spatial.transform import Rotation
from multipath.planning.floating import pose_coordinates, coordinates_pose
from multipath.planning.floating_diversity import pose_velocity


def nearest_entry_velocity(start, goal_command, ticks, dt, limits):
    """Project measured dq onto the bridge's nominal reachability envelope.

    This is a necessary boundary condition, not a collision or physics proof.
    No projected state is assigned to the simulator.
    """
    n=ticks+1
    q0=pose_coordinates(start['wrist_xyz_wxyz'])
    q1=pose_coordinates(goal_command[:7],q0)
    v0=pose_velocity(start['wrist_xyz_wxyz'],start['wrist_twist'],q0)
    v1=pose_velocity(goal_command[:7],goal_command[19:25],q1)
    d1=np.diff(np.eye(n),axis=0);d2=np.diff(np.eye(n),n=2,axis=0)
    derivative=np.vstack((d1,-d1,d2,-d2))
    matrix=np.c_[derivative,np.zeros(len(derivative))]
    slope=np.zeros(n);slope[:2]=[-1/dt,1/dt]
    matrix=np.vstack((matrix,np.r_[slope,-1],np.r_[-slope,-1]))
    equal=np.zeros((3,n+1));equal[0,0]=equal[1,n-1]=equal[2,n-2]=1
    nearest=v0.copy()
    for j in range(6):
        rhs=np.r_[np.full(2*(n-1),limits['velocity_limits'][j]*dt),
                  np.full(2*(n-2),limits['acceleration_limits'][j]*dt**2),v0[j],-v0[j]]
        result=linprog(np.r_[np.zeros(n),1.],A_ub=matrix,b_ub=rhs,A_eq=equal,
            b_eq=[q0[j],q1[j],q1[j]-v1[j]*dt],
            bounds=[(None,None)]*n+[(0,None)],method='highs')
        if not result.success:
            raise RuntimeError(f'Outgoing bridge also needs an Anchor pose correction in coordinate {j}')
        nearest[j]=slope@result.x[:n]
    return nearest-v0


def terminal_velocity_residual(commands, delta, duration_s, dt):
    """A local Hermite residual: unchanged final pose, changed final tangent.

    The window is sized from synthesis context. Fingers, core duration and
    source timing are unchanged. Only physical validation can accept it.
    """
    original=np.asarray(commands);candidate=original.copy()
    # Identity shooting must replay the exact validated drive/wrench pair.
    # A quaternion -> rotation-vector -> quaternion round trip changes bits,
    # so the caller would otherwise discard the native controller program.
    if not np.any(np.asarray(delta)):
        return candidate
    ticks=min(len(candidate)-1,max(3,round(duration_s/dt)))
    duration=ticks*dt
    phase=np.clip((np.arange(len(candidate))-(len(candidate)-1-ticks))/ticks,0,1)
    residual=duration*(phase**3-phase**2)[:,None]*np.asarray(delta)
    coordinates=[];near=None
    for pose in original[:,:7]:
        near=pose_coordinates(pose,near);coordinates.append(near)
    active=np.flatnonzero(phase>0)
    candidate[active,:7]=np.asarray([coordinates_pose(coordinates[i]+residual[i]) for i in active])
    candidate[1:,19:22]+=np.diff(residual[:,:3],axis=0)/dt
    old=Rotation.from_quat(original[:,[4,5,6,3]])
    new=Rotation.from_quat(candidate[:,[4,5,6,3]])
    candidate[1:,22:25]+=((new[1:]*new[:-1].inv()).as_rotvec()-
                            (old[1:]*old[:-1].inv()).as_rotvec())/dt
    return candidate
