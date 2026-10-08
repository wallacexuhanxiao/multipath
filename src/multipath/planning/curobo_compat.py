"""cuRobo compatibility for empty sphere pairs and 16-bit CUDA addresses."""
def install_curobo_compatibility():
    import torch
    from curobo.curobolib.kinematics import kinematics_fused_cu
    forward=kinematics_fused_cu.forward
    if not getattr(forward,'_multipath_safe_sphere_addresses',False):
        def safe_forward(*args):
            count=args[15]
            # The installed forward kernel stores sphere_index * 4 in
            # int16_t. Index 8192 overflows and can corrupt unrelated tensors.
            # Each call stays in range; Python indexing is wide. All spheres
            # remain present, and the native backward already uses int offsets.
            if count<=8192:return forward(*args)
            spheres=args[6].reshape(-1,4);output=args[2]
            for begin in range(0,count,8192):
                end=min(count,begin+8192);chunk=list(args)
                chunk[2]=output.new_empty((args[13],end-begin,4))
                chunk[6]=spheres[begin:end].contiguous().reshape(-1)
                chunk[11]=args[11][begin:end].contiguous();chunk[15]=end-begin
                value=forward(*chunk)
                output[:,begin:end].copy_(value[2])
            return value[0],value[1],output,value[3]
        safe_forward._multipath_safe_sphere_addresses=True
        kinematics_fused_cu.forward=safe_forward
    from curobo.cuda_robot_model.cuda_robot_generator import CudaRobotGenerator
    original=CudaRobotGenerator._create_self_collision_thread_data
    if getattr(original,'_multipath_empty_pairs',False):return

    def create(self,threshold):
        # A single virtual wrist stores the full hand/carried-object surface
        # on one link. All its self-collision pairs are already ignored.
        # Upstream nevertheless visits every -inf pair in Python, taking
        # minutes after boundary refinement. Return its exact empty result.
        if torch.eq(threshold,-torch.inf).all().item():
            return (threshold.new_full((2*32*512,),-1,dtype=torch.int16),0,True,2)
        return original(self,threshold)

    create._multipath_empty_pairs=True
    CudaRobotGenerator._create_self_collision_thread_data=create
