"""Mounted Franka/XHand backend. State assignment is initialisation only."""
import hashlib
import json
import numpy as np
from scipy.spatial.transform import Rotation
from multipath.mounted_xhand import assemble, configure_drives, ActualMeshRecorder


def pose_matrix(pose):
    p=np.asarray(pose); t=np.eye(4)
    t[:3,:3]=Rotation.from_quat(p[[4,5,6,3]]).as_matrix();t[:3,3]=p[:3]
    return t


class MountedScene:
    dt=1/120
    def __init__(self,args,manifest,out):
        import yaml,trimesh
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import FixedCuboid
        from isaacsim.core.prims import SingleRigidPrim, RigidPrim
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.utils.types import ArticulationAction
        from curobo.types.base import TensorDeviceType
        from curobo.wrap.reacher.ik_solver import IKSolver,IKSolverConfig
        from curobo.geom.sdf.world import CollisionCheckerType
        self.Action=ArticulationAction
        self.out=out;self.args=args;self.manifest=manifest
        self.config=yaml.safe_load(args.robot_yaml.read_text())['robot_cfg']
        self.mount=Rotation.from_quat(manifest['mount_xyzw'])
        self.meshes={r:trimesh.load(args.object_cache/o['id']/(o['id']+'_m.obj'),force='mesh') for r,o in manifest['initial_objects'].items()}
        initial=manifest['initial_objects']
        z=min((pose_matrix(o['pose'])[:3,:3]@self.meshes[r].vertices.T).T[:,2].min()+o['pose'][2] for r,o in initial.items())-.001
        self.table={'dims':[.95,1.,.04],'pose':[.57,0,float(z-.02),1,0,0,0]}
        self.world=World(stage_units_in_meters=1.,physics_dt=self.dt,rendering_dt=1/30)
        self.world.scene.add_default_ground_plane()
        self.world.scene.add(FixedCuboid(prim_path='/World/Table',name='table',position=np.array(self.table['pose'][:3]),scale=np.array(self.table['dims'])))
        self.robot,self.wrist,self.flange=assemble(self.world,args.xhand_urdf,manifest['mount_xyzw'])
        self.objects={}
        for role,obj in initial.items():
            path='/World/'+role;oid=obj['id']
            add_reference_to_stage(str(args.object_cache/oid/(oid+'.usd')),path)
            self.objects[role]=self.world.scene.add(SingleRigidPrim(prim_path=path,name=role,position=np.array(obj['pose'][:3]),orientation=np.array(obj['pose'][3:])))
        # Per rigid XHand link -> tool forces, so support contact cannot masquerade as a grasp.
        from pxr import UsdPhysics
        contact_links=[str(p.GetPath()) for p in self.world.stage.Traverse()
                       if str(p.GetPath()).startswith('/xhand_right/') and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        self.contacts=self.world.scene.add(RigidPrim(prim_paths_expr=contact_links,name='hand_tool_contacts',
                         track_contact_forces=True,contact_filter_prim_paths_expr=[['/World/tool'] for _ in contact_links],max_contact_count=1024))
        self.world.reset()
        self.arm,self.fingers=configure_drives(self.robot)
        self.tensor=TensorDeviceType(device='cuda:0')
        self.ik=IKSolver(IKSolverConfig.load_from_robot_config(self.config,{'cuboid':{'table':self.table}},tensor_args=self.tensor,num_seeds=40,
               position_threshold=.005,rotation_threshold=.05,self_collision_check=True,self_collision_opt=True,
               collision_checker_type=CollisionCheckerType.PRIMITIVE,use_cuda_graph=False))
        self.velocity=np.array([2.175]*4+[2.61]*3)
        # cuRobo nominal time-parameterization budget, NOT an actual-state
        # actuator acceleration rating. Keep it for fitting and diagnostics.
        self.acceleration=np.array(self.config['kinematics']['cspace'].get('max_acceleration',15.))
        if self.acceleration.ndim:self.acceleration=self.acceleration[:7]
        q=np.asarray(self.robot.get_joint_positions()).copy()
        # Robot's nominal retract is an embodiment property, not a human trajectory.
        q[self.arm]=np.array(self.config['kinematics']['cspace']['retract_config'])[:7]
        q[self.fingers]=0.
        self.robot.set_joint_positions(q);self.robot.set_joint_velocities(np.zeros_like(q))
        self.robot.apply_action(self.Action(joint_positions=q))
        q[self.arm]=self.collision_free_ready(q[self.arm])
        self.robot.set_joint_positions(q)
        self.robot.apply_action(self.Action(joint_positions=q))
        for _ in range(120):self.world.step(render=False)
        self.qcommand=q.copy();self.velocity_command=np.zeros_like(q);self.ticks=0;self.records=[];self.max_velocity_ratio=0.;self.max_acceleration_ratio=0.;self.previous_dq=np.asarray(self.robot.get_joint_velocities())[self.arm].copy();self.recorder=ActualMeshRecorder(self.world.stage,out)
        self.scene_id=hashlib.sha256(json.dumps(initial,sort_keys=True).encode()).hexdigest()
        self.capture('INITIAL',0)

    def collision_free_ready(self,nominal):
        """Choose initial configuration against the mounted geometry, before t=0."""
        planner=self.planner(False)
        limits=np.asarray(self.robot._articulation_view.get_dof_limits()).reshape(-1,2)[self.arm]
        candidates=[np.array(nominal).copy()]
        for radius in (.25,.5,.75):
            for j in range(7):
                for direction in (-1,1):
                    q=np.array(nominal).copy();q[j]+=direction*radius
                    candidates.append(np.clip(q,limits[:,0]+.01,limits[:,1]-.01))
        for q in candidates:
            if self.collision_valid(planner,q):
                self.ready_search={'candidate_count':len(candidates),'nominal':np.array(nominal).tolist(),'selected':q.tolist()}
                return q
        raise RuntimeError('Mounted embodiment has no collision-free ready pose near nominal retract')

    def collision_valid(self,planner,q):
        from curobo.types.robot import JointState
        state=JointState.from_position(self.tensor.to_device(np.array(q).copy())[None],joint_names=planner.joint_names)
        try:return bool(planner.check_start_state(state)[0])
        finally:
            # cuRobo 0.7.7 leaves self collision disabled on its world-collision early return.
            planner.rollout_fn.primitive_collision_constraint.enable_cost()
            planner.rollout_fn.robot_self_collision_constraint.enable_cost()

    def contact_state(self):
        from multipath.validation.physics import hand_tool_contact
        return hand_tool_contact(self.contacts.prim_paths,
                                 self.contacts.get_contact_force_matrix(dt=self.dt))

    def state(self):
        q=np.asarray(self.robot.get_joint_positions());dq=np.asarray(self.robot.get_joint_velocities())
        wp,wq=self.wrist.get_world_pose();objects={}
        for role,obj in self.objects.items():
            p,r=obj.get_world_pose()
            pose=np.r_[p,r];velocity=np.r_[obj.get_linear_velocity(),obj.get_angular_velocity()]
            if not np.isfinite(pose).all() or not np.isfinite(velocity).all():
                raise RuntimeError(f'Non-finite measured object state: {role}')
            objects[role]={'pose':pose.tolist(),'velocity':velocity.tolist()}
        measured_effort=np.asarray(self.robot.get_measured_joint_efforts())
        relative=np.linalg.inv(pose_matrix(np.r_[wp,wq]))@pose_matrix(objects['tool']['pose'])
        return {'arm_q':q[self.arm].tolist(),'arm_dq':dq[self.arm].tolist(),
                'measured_joint_efforts':measured_effort[np.r_[self.arm,self.fingers]].tolist(),
                'hand_q':q[self.fingers].tolist(),'hand_dq':dq[self.fingers].tolist(),
                'objects':objects,'wrist_xyz_wxyz':np.r_[wp,wq].tolist(),
                'hand_object_transform':relative.tolist(),'contact':self.contact_state(),
                'scene_id':self.scene_id,'physically_acquired_state':True,
                'time_s':self.ticks*self.dt,'state_origin':'continuous_PhysX_rollout'}

    def capture(self,mode,frame):
        s=self.state()
        self.records.append(dict(s,mode=mode,source_frame=frame,arm_q=s['arm_q'],finger_q=s['hand_q'],
                         command=self.qcommand.tolist(),command_velocity=self.velocity_command.tolist(),
                         command_arm_q=self.qcommand[self.arm].tolist(),command_arm_dq=self.velocity_command[self.arm].tolist(),
                         tool_xyz_wxyz=s['objects']['tool']['pose'],target_xyz_wxyz=s['objects']['target']['pose']))
        self.recorder.capture()

    def step(self,arm,hand,mode,frame,arm_velocity):
        self.velocity_command[:]=0.
        self.velocity_command[self.arm]=arm_velocity
        self.qcommand[self.arm]=arm;self.qcommand[self.fingers]=hand
        self.robot.apply_action(self.Action(joint_positions=self.qcommand,joint_velocities=self.velocity_command))
        self.world.step(render=False);self.ticks+=1
        q_all=np.asarray(self.robot.get_joint_positions());dq_all=np.asarray(self.robot.get_joint_velocities())
        if not np.isfinite(q_all).all() or not np.isfinite(dq_all).all():
            raise RuntimeError('Non-finite measured articulation state')
        dq=dq_all[self.arm]
        self.max_velocity_ratio=max(self.max_velocity_ratio,float(np.max(np.abs(dq)/self.velocity)))
        self.max_acceleration_ratio=max(self.max_acceleration_ratio,float(np.max(np.abs(dq-self.previous_dq)/self.dt/self.acceleration)))
        self.previous_dq=dq.copy()
        if self.ticks%round(1/(self.dt*self.manifest['fps']))==0:self.capture(mode,frame)
        if self.max_velocity_ratio>1.05:
            raise RuntimeError(f'Measured actuator velocity exceeded: velocity={self.max_velocity_ratio:.3f}; dq={dq.tolist()}')

    def solve(self,wrist_pose,seed):
        """Mounted local retargeting follows the preceding IK branch."""
        from scipy.optimize import least_squares
        pose=np.asarray(wrist_pose);rotation=Rotation.from_quat(pose[[4,5,6,3]])*self.mount.inv()
        limits=np.asarray(self.robot._articulation_view.get_dof_limits()).reshape(-1,2)[self.arm]
        def fk(q):
            state=self.ik.kinematics.get_state(self.tensor.to_device(q.copy())[None])
            return state.ee_position.detach().cpu().numpy()[0],Rotation.from_quat(state.ee_quaternion.detach().cpu().numpy()[0][[1,2,3,0]])
        def residual(q):
            position,r=fk(q)
            return np.r_[position-pose[:3],.15*(rotation*r.inv()).as_rotvec(),.001*(q-seed)]
        fit=least_squares(residual,np.clip(seed,limits[:,0]+1e-6,limits[:,1]-1e-6),
                          bounds=(limits[:,0],limits[:,1]),diff_step=2e-3,max_nfev=60,
                          xtol=1e-7,ftol=1e-7,gtol=1e-7)
        position,r=fk(fit.x)
        error=float(np.linalg.norm(position-pose[:3]));angle=float((rotation*r.inv()).magnitude())
        # Same tolerances as the original cuRobo IK, not relaxed after failure.
        if error>.005 or angle>.05:
            raise RuntimeError(f'Local Anchor IK failed: {error:.5f}m, {angle:.5f}rad')
        return fit.x

    def planner(self,carry):
        from multipath.planning.mounted_collision import mounted_collision_robot
        from curobo.wrap.reacher.motion_gen import MotionGen,MotionGenConfig
        from curobo.geom.sdf.world import CollisionCheckerType
        cfg,_=mounted_collision_robot(self.config,self.args.robot_yaml,self.world,self.flange,self.objects,self.meshes,carry)
        live={'cuboid':{'table':self.table}}
        for role in (['target'] if carry else ['tool','target']):
            p,q=self.objects[role].get_world_pose();bounds=self.meshes[role].bounds
            r=Rotation.from_quat(np.array(q)[[1,2,3,0]])
            live['cuboid'][role]={'dims':(bounds[1]-bounds[0]).tolist(),'pose':np.r_[p+r.apply(bounds.mean(0)),q].tolist()}
        return MotionGen(MotionGenConfig.load_from_robot_config(cfg,live,tensor_args=self.tensor,
                         interpolation_dt=self.dt,self_collision_check=True,
                         collision_checker_type=CollisionCheckerType.PRIMITIVE,use_cuda_graph=False))

    def save(self):
        self.recorder.save()
        (self.out/'actual_trace.json').write_text(json.dumps(self.records))
