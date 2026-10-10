"""A second physical XHand sharing the existing scene and physics clock."""
import copy,os,tempfile
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from multipath.xhand import finger_joint_names


class SupportingHand:
    embodiment='floating_xhand'
    def __init__(self,scene,side,urdf,pose,hand,object_role):
        from isaacsim.asset.importer.urdf import _urdf
        from isaacsim.core.api.robots import Robot
        from isaacsim.core.prims import SingleRigidPrim,RigidPrim
        from pxr import UsdPhysics
        self.scene,self.side,self.object_role=scene,side,object_role
        self.world,self.cfg,self.dt,self.table=scene.world,scene.cfg,scene.dt,scene.table
        self.Action=scene.Action;self.args=copy.copy(scene.args);self.args.xhand_urdf=Path(urdf)
        # Existing local/planning helpers call the manipulated role 'tool'.
        # This view maps that role to this hand's declared object, not a copy.
        other=next(r for r in scene.objects if r!=object_role)
        self.objects={'tool':scene.objects[object_role],'target':scene.objects[other]}
        self.meshes={'tool':scene.meshes[object_role],'target':scene.meshes[other]}
        self.manifest=dict(scene.manifest,initial_objects={
            'tool':scene.manifest['initial_objects'][object_role],
            'target':scene.manifest['initial_objects'][other]})
        self.manifest['retarget_details']=scene.manifest.get('supporting_retarget_details',{})
        self.manifest['interaction_constraints']={}
        cfg=_urdf.ImportConfig();cfg.set_fix_base(False);cfg.set_merge_fixed_joints(False);cfg.set_self_collision(False)
        importer=_urdf.acquire_urdf_interface();urdf=Path(urdf).resolve();previous=Path.cwd()
        before={str(p.GetPath()) for p in self.world.stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)}
        try:
            os.chdir(tempfile.mkdtemp(prefix='multipath-'+side+'-'))
            parsed=importer.parse_urdf(str(urdf.parent),urdf.name,cfg)
            self.root=str(importer.import_robot(str(urdf.parent),urdf.name,parsed,cfg,''))
        finally:os.chdir(previous)
        self.colliders={str(p.GetPath()) for p in self.world.stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)}-before
        for prim in self.world.stage.Traverse():
            if str(prim.GetPath()).startswith(self.root) and prim.IsA(UsdPhysics.RevoluteJoint):
                UsdPhysics.DriveAPI.Apply(prim,'angular').CreateTypeAttr('force')
        self.initial_pose=np.asarray(pose).copy();self.initial_hand=np.asarray(hand).copy()
        self.robot=self.world.scene.add(Robot(prim_path=self.root,name=side+'_xhand',position=self.initial_pose[:3],orientation=self.initial_pose[3:]))
        self.wrist=SingleRigidPrim(prim_path=self.root+'/'+side+'_hand_link',name=side+'_wrist')
        links=[str(p.GetPath()) for p in self.world.stage.Traverse()
               if str(p.GetPath()).startswith(self.root+'/') and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        self.contacts=self.world.scene.add(RigidPrim(prim_paths_expr=links,name=side+'_object_contacts',track_contact_forces=True,
            contact_filter_prim_paths_expr=[['/World/'+object_role] for _ in links],max_contact_count=1024))
        from pxr import UsdShade
        material=UsdShade.Material(self.world.stage.GetPrimAtPath('/World/MultipathHandToolMaterial'))
        if material:
            for path in self.colliders:
                prim=self.world.stage.GetPrimAtPath(path);binding=UsdShade.MaterialBindingAPI(prim)
                authored,_=binding.ComputeBoundMaterial(materialPurpose='physics')
                if not authored or not authored.GetPrim().HasAPI(UsdPhysics.MaterialAPI):
                    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material,materialPurpose='physics')
        self.pending_command=None

    @property
    def ticks(self):return self.scene.ticks

    @property
    def scene_id(self):return self.scene.scene_id

    @property
    def out(self):return self.scene.out

    @property
    def restore_count(self):return self.scene.restore_count

    @property
    def tool_target_contact_steps(self):return self.scene.tool_target_contact_steps

    @property
    def physical_properties(self):
        return dict(hand_mass_kg=float(np.asarray(self.av.get_masses()).sum()),
                    tool_mass_kg=float(self.objects['tool'].get_mass()))

    @property
    def records(self):
        return [dict(r['hands'][self.side],mode=r['mode'],source_frame=r['source_frame'])
                for r in self.scene.records]

    def initialize(self):
        self.wrist.initialize();view=self.robot._articulation_view;self.av=view._physics_view
        self.fingers=np.asarray([list(self.robot.dof_names).index(n) for n in finger_joint_names(self.side)])
        cfg=self.cfg['controller']
        self.hand_limits=np.asarray(view.get_dof_limits()).reshape(-1,2)[self.fingers]
        from multipath.anchors.synthesis import hand_limits_from_urdf
        self.hand_velocity=hand_limits_from_urdf(self)
        self.hand_drive_stiffness=np.broadcast_to(np.asarray(cfg['hand_stiffness'],dtype=float),(12,)).copy()
        view.set_gains(kps=self.hand_drive_stiffness[None],kds=np.broadcast_to(cfg['hand_damping'],(1,12)),joint_indices=self.fingers)
        view.set_max_efforts(np.broadcast_to(cfg['hand_max_effort'],(1,12)),joint_indices=self.fingers)
        pose=self.initial_pose;self.robot.set_world_pose(position=pose[:3],orientation=pose[3:])
        self.robot.set_linear_velocity(np.zeros(3));self.robot.set_angular_velocity(np.zeros(3))
        q=np.zeros(self.robot.num_dof);q[self.fingers]=np.clip(self.initial_hand,*self.hand_limits.T)
        self.robot.set_joint_positions(q);self.robot.set_joint_velocities(np.zeros_like(q))
        self.hand_command=q[self.fingers].copy();self.hand_command_velocity=np.zeros(12)
        self.command_pose=pose.copy();self.command_twist=np.zeros(6);self.last_wrench=np.zeros(6)
        self.wrench_bias=np.zeros(6);self.transport_load=False;self.pending_command=None

    def root_motion(self):
        from multipath.execution.floating_scene import FloatingScene
        return FloatingScene.root_motion(self)

    def _drive(self,pose,hand,twist):
        from multipath.execution.floating_scene import FloatingScene
        return FloatingScene._drive(self,pose,hand,twist)

    def state(self):
        from multipath.validation.physics import hand_tool_contact
        from multipath.execution.mounted_scene import pose_matrix
        pose,twist=self.root_motion();p,q=self.objects['tool'].get_world_pose()
        object_pose=np.r_[p,q];objects={}
        for role,obj in self.objects.items():
            p,q=obj.get_world_pose();objects[role]={'pose':np.r_[p,q].tolist(),
                'velocity':np.r_[obj.get_linear_velocity(),obj.get_angular_velocity()].tolist()}
        result=dict(embodiment=self.embodiment,wrist_xyz_wxyz=pose.tolist(),wrist_twist=twist.tolist(),
            hand_q=np.asarray(self.robot.get_joint_positions())[self.fingers].tolist(),
            hand_dq=np.asarray(self.robot.get_joint_velocities())[self.fingers].tolist(),objects=objects,
            hand_object_transform=(np.linalg.inv(pose_matrix(pose))@pose_matrix(object_pose)).tolist(),
            contact=hand_tool_contact(self.contacts.prim_paths,self.contacts.get_contact_force_matrix(dt=self.dt)),
            command_wrist=self.command_pose.tolist(),command_twist=self.command_twist.tolist(),
            command_hand_q=self.hand_command.tolist(),command_hand_dq=self.hand_command_velocity.tolist(),
            controller_wrench_bias=self.wrench_bias.tolist(),applied_wrench=self.last_wrench.tolist(),
            object_role=self.object_role,side=self.side,scene_id=self.scene.scene_id,
            time_s=self.scene.ticks*self.dt,physically_acquired_state=True,state_origin='continuous_PhysX_rollout')
        if not np.isfinite(np.r_[pose,twist,result['hand_q'],result['hand_dq']]).all():
            raise RuntimeError('Non-finite supporting-hand PhysX state')
        return result

    def snapshot(self):
        full=self.scene.snapshot()
        return dict(self.state(),entry_origin='predecessor_rollout',source_provenance=self.manifest['provenance'],
                    entry_provenance=full['entry_provenance'],full_scene_snapshot=full)

    def begin_trial(self,out,state,mesh=False):
        self.scene.begin_trial(out,state['full_scene_snapshot'],mesh=mesh)

    def begin_recording(self,out,mesh=False):self.scene.begin_recording(out,mesh=mesh)

    def save(self):self.scene.save()

    def step_control(self,command,mode,frame):
        if not hasattr(self,'primary_control'):
            raise RuntimeError('Supporting synthesis requires a physically executed primary-hand controller')
        primary,wrench=self.primary_control(frame)
        self.scene.wrench_bias=wrench
        self.scene.step_control(primary,mode,frame,supporting_commands={self.side:command})

    def check_transition(self,*args,**kwargs):
        from multipath.execution.floating_scene import FloatingScene
        return FloatingScene.check_transition(self,*args,**kwargs)

    def restore_development(self,state):
        if not state.get('physically_acquired_state') or state['scene_id']!=self.scene.scene_id:
            raise ValueError('Supporting hand restore requires an actual same-scene snapshot')
        pose=np.asarray(state['wrist_xyz_wxyz']);twist=np.asarray(state['wrist_twist'])
        self.robot.set_world_pose(position=pose[:3],orientation=pose[3:])
        com=np.asarray(self.av.get_coms())[0,0,:3]
        self.robot.set_linear_velocity(twist[:3]+np.cross(twist[3:],Rotation.from_quat(pose[[4,5,6,3]]).apply(com)))
        self.robot.set_angular_velocity(twist[3:])
        q=np.asarray(self.robot.get_joint_positions()).copy();dq=np.zeros_like(q)
        q[self.fingers]=state['hand_q'];dq[self.fingers]=state['hand_dq']
        self.robot.set_joint_positions(q);self.robot.set_joint_velocities(dq)
        self.command_pose=np.asarray(state['command_wrist']);self.command_twist=np.asarray(state['command_twist'])
        self.hand_command=np.asarray(state['command_hand_q']);self.hand_command_velocity=np.asarray(state['command_hand_dq'])
        self.wrench_bias=np.asarray(state['controller_wrench_bias']);self.transport_load=False;self.pending_command=None
        q[self.fingers]=self.hand_command;dq[:]=0;dq[self.fingers]=self.hand_command_velocity
        self.robot.apply_action(self.Action(joint_positions=q,joint_velocities=dq))
