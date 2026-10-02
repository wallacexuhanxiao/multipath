"""Shared nominal control check; physical rollout remains authoritative."""
import numpy as np


def control_feasibility(control,start_q,start_dq,dt,limits,velocity_limits,acceleration_limits,*,start_hand_q=None,hand_velocity_limits=None):
    control=np.asarray(control);limits=np.asarray(limits)
    if control.ndim!=2 or control.shape[1]!=26 or not len(control) or not np.isfinite(control).all():
        return {'feasible':False,'error':'invalid or non-finite mounted commands'}
    velocity=np.diff(np.vstack((start_q,control[:,:7])),axis=0)/dt
    acceleration=np.diff(np.vstack((start_dq,velocity)),axis=0)/dt
    # Isaac's limits are float32. Permit representation error only, not a
    # tunable physical margin. E.g. an interpolated zero can be -2.5e-19.
    tolerance=8*np.finfo(np.float32).eps*np.maximum(1,np.max(np.abs(limits),axis=1))
    excess=np.maximum(limits[:,0]-control[:,:19],control[:,:19]-limits[:,1])
    bad=np.argwhere(excess>tolerance)
    vr=float(np.max(abs(velocity)/velocity_limits));ar=float(np.max(abs(acceleration)/acceleration_limits))
    feedforward_error=float(np.max(abs(control[:,19:26]-velocity)))
    report={'feedforward_velocity_error_rad_s':feedforward_error,'velocity_ratio':vr,'acceleration_ratio':ar,
            'joint_limit_violations':[{'tick':int(i),'joint_index':int(j),'q':float(control[i,j]),
                 'lower':float(limits[j,0]),'upper':float(limits[j,1]),'excess_rad':float(excess[i,j])} for i,j in bad],
            'joint_limit_roundoff_tolerance_rad':tolerance.tolist()}
    hvr=0.
    if hand_velocity_limits is not None:
        hv=np.diff(np.vstack((start_hand_q,control[:,7:19])),axis=0)/dt
        hvr=float(np.max(abs(hv)/hand_velocity_limits))
        report['hand_velocity_ratio']=hvr
    report['feasible']=bool(np.isfinite(control).all() and not len(bad) and vr<=1.0001 and ar<=1.0001 and hvr<=1.00001 and feedforward_error<=1e-5)
    return report
