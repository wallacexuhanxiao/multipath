"""Floating XHand PhysX backend; wrist commands are external wrenches, not poses.

Only initialisation and explicitly marked local-development restores assign state.
The continuous executor uses step_control exclusively.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from multipath.execution.mounted_scene import pose_matrix
from multipath.mounted_xhand import ActualMeshRecorder
from multipath.xhand import RIGHT_XHAND_FINGER_JOINT_NAMES


class FloatingScene:
    dt = 1/120
    embodiment = 'floating_xhand'

    def __init__(self, args, manifest, out, initial_wrist, initial_hand=None,*,supporting_initial=None):
        import os, tempfile, omni, trimesh
        from isaacsim.asset.importer.urdf import _urdf
        from isaacsim.core.api import World
        from isaacsim.core.api.robots import Robot
        from isaacsim.core.api.objects import FixedCuboid
        from isaacsim.core.prims import SingleRigidPrim, RigidPrim
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.utils.types import ArticulationAction
        from pxr import Usd,UsdGeom,UsdPhysics
        self.args,self.manifest,self.out=args,manifest,Path(out)
        from multipath.planning.timing import physics_ticks_per_frame
        self.tpf=physics_ticks_per_frame(manifest['fps'],self.dt)
        self.out.mkdir(parents=True,exist_ok=True)
        self.cfg=json.loads(Path(args.floating_config).read_text())
        self.Action=ArticulationAction
        self.meshes={r:trimesh.load(args.object_cache/o['id']/(o['id']+'_m.obj'),force='mesh') for r,o in manifest['initial_objects'].items()}
        initial=manifest['initial_objects']
        z=min((pose_matrix(o['pose'])[:3,:3]@self.meshes[r].vertices.T).T[:,2].min()+o['pose'][2] for r,o in initial.items())-.001
        self.table={'dims':[.95,1.,.04],'pose':[.57,0,float(z-.02),1,0,0,0]}
        self.world=World(stage_units_in_meters=1.,physics_dt=self.dt,rendering_dt=1/30)
        self.world.scene.add_default_ground_plane()
        self.world.scene.add(FixedCuboid(prim_path='/World/Table',name='table',position=np.array(self.table['pose'][:3]),scale=np.array(self.table['dims'])))
        existing_colliders={str(p.GetPath()) for p in self.world.stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)}
        cfg=_urdf.ImportConfig();cfg.set_fix_base(False);cfg.set_merge_fixed_joints(False);cfg.set_self_collision(False)
        importer=_urdf.acquire_urdf_interface();urdf=Path(args.xhand_urdf).resolve()
        previous=Path.cwd()
        try:
            os.chdir(tempfile.mkdtemp(prefix='multipath-floating-'))
            parsed=importer.parse_urdf(str(urdf.parent),urdf.name,cfg)
            self.root=str(importer.import_robot(str(urdf.parent),urdf.name,parsed,cfg,''))
        finally:os.chdir(previous)
        # URDF collider geometry may live in a shared /colliders library,
        # outside the articulation root. Track what this import created.
        hand_colliders={str(p.GetPath()) for p in self.world.stage.Traverse()
                        if p.HasAPI(UsdPhysics.CollisionAPI)}-existing_colliders
        self.robot=self.world.scene.add(Robot(prim_path=self.root,name='xhand',position=np.asarray(initial_wrist[:3]),orientation=np.asarray(initial_wrist[3:])))
        self.wrist=SingleRigidPrim(prim_path=self.root+'/right_hand_link',name='wrist')
        for prim in self.world.stage.Traverse():
            if str(prim.GetPath()).startswith(self.root) and prim.IsA(UsdPhysics.RevoluteJoint):
                UsdPhysics.DriveAPI.Apply(prim,'angular').CreateTypeAttr('force')
        self.fixed_environment_roles=set(self.cfg.get('physics',{}).get('fixed_environment_roles',[]))
        if self.fixed_environment_roles-set(initial) or 'tool' in self.fixed_environment_roles:
            raise ValueError('Fixed environment roles must name uncontrolled scene objects')
        self.objects={}
        for role,obj in initial.items():
            path='/World/'+role;oid=obj['id']
            add_reference_to_stage(str(args.object_cache/oid/(oid+'.usd')),path)
            self.objects[role]=self.world.scene.add(SingleRigidPrim(prim_path=path,name=role,position=np.array(obj['pose'][:3]),orientation=np.array(obj['pose'][3:])))
            if role in self.fixed_environment_roles:
                # An omitted supporting hand is an explicit scene fixture,
                # not a reference animation. The manipulated tool stays free.
                root=self.world.stage.GetPrimAtPath(path)
                UsdPhysics.RigidBodyAPI.Apply(root).CreateKinematicEnabledAttr(True)
                for prim in Usd.PrimRange(root):
                    if prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.CollisionAPI):
                        # Fixed/kinematic bodies permit triangle collision.
                        # Match the actual support surface to the planner mesh;
                        # dynamic convex decomposition can fill plate cavities.
                        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')
        physics=self.cfg.get('physics')
        if physics is not None:
            from pxr import UsdShade
            material=UsdShade.Material.Define(self.world.stage,'/World/MultipathHandToolMaterial')
            api=UsdPhysics.MaterialAPI.Apply(material.GetPrim())
            api.CreateStaticFrictionAttr(float(physics['hand_tool_static_friction']))
            api.CreateDynamicFrictionAttr(float(physics['hand_tool_dynamic_friction']))
            self.hand_tool_material_colliders=[]
            for prim in self.world.stage.Traverse():
                path=str(prim.GetPath())
                if not prim.HasAPI(UsdPhysics.CollisionAPI) or not (path in hand_colliders or path.startswith('/World/tool/')):continue
                binding=UsdShade.MaterialBindingAPI(prim)
                authored,_=binding.ComputeBoundMaterial(materialPurpose='physics')
                if authored and authored.GetPrim().HasAPI(UsdPhysics.MaterialAPI):continue
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material,materialPurpose='physics')
                self.hand_tool_material_colliders.append(path)
        links=[str(p.GetPath()) for p in self.world.stage.Traverse() if str(p.GetPath()).startswith(self.root+'/') and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        self.contacts=self.world.scene.add(RigidPrim(prim_paths_expr=links,name='hand_tool_contacts',track_contact_forces=True,
            contact_filter_prim_paths_expr=[['/World/tool'] for _ in links],max_contact_count=1024))
        self.tool_target_contacts=None
        if 'target' in self.objects:
            self.tool_target_contacts=self.world.scene.add(RigidPrim(prim_paths_expr=['/World/tool'],name='tool_target_contacts',
                track_contact_forces=True,contact_filter_prim_paths_expr=[['/World/target']],max_contact_count=1024))
        self.tool_target_contact_steps=0;self.tool_target_impulse=0.
        # Import both articulations before the first PhysX reset. Adding a
        # robot after views are live invalidates their native handles.
        self.supporting_hands={}
        for side,definition in (supporting_initial or {}).items():
            if side!='left' or definition['object_role'] in self.fixed_environment_roles:
                raise ValueError('A supporting XHand requires a dynamic assigned object')
            from multipath.execution.supporting_hand import SupportingHand
            self.supporting_hands[side]=SupportingHand(self,side,definition['urdf'],definition['pose'],
                                                     definition['hand'],definition['object_role'])
        if self.supporting_hands:self.embodiment='floating_xhand_bimanual'
        self.world.reset();self.wrist.initialize()
        for support in self.supporting_hands.values():support.initialize()
        names=list(self.robot.dof_names);self.fingers=np.array([names.index(n) for n in RIGHT_XHAND_FINGER_JOINT_NAMES])
        view=self.robot._articulation_view
        controller=self.cfg['controller']
        self.hand_drive_stiffness=np.broadcast_to(np.asarray(controller.get('hand_stiffness',5.),dtype=float),(12,)).copy()
        self.hand_drive_damping=np.broadcast_to(np.asarray(controller.get('hand_damping',.5),dtype=float),(12,)).copy()
        self.hand_effort_limits=np.broadcast_to(np.asarray(controller.get('hand_max_effort',3.),dtype=float),(12,)).copy()
        view.set_gains(kps=self.hand_drive_stiffness[None],kds=self.hand_drive_damping[None],joint_indices=self.fingers)
        view.set_max_efforts(self.hand_effort_limits[None],joint_indices=self.fingers)
        self.hand_limits=np.asarray(view.get_dof_limits()).reshape(-1,2)[self.fingers]
        from multipath.anchors.synthesis import hand_limits_from_urdf
        self.hand_velocity=hand_limits_from_urdf(self)
        # The free ready state uses the first local context's pregrasp posture.
        # Carrying a zero/open hand across RELAXED otherwise forces the whole
        # pregrasp closure into the short padding and changes Anchor entry.
        self.hand_command=np.clip(np.zeros(12) if initial_hand is None else np.asarray(initial_hand),
                                  self.hand_limits[:,0],self.hand_limits[:,1])
        q=np.zeros(self.robot.num_dof);q[self.fingers]=self.hand_command
        self.robot.set_joint_positions(q);self.robot.set_joint_velocities(np.zeros_like(q))
        self.av=view._physics_view
        self.physical_properties={'hand_mass_kg':float(np.asarray(self.av.get_masses()).sum()),
                                  'tool_mass_kg':float(self.objects['tool'].get_mass()),
                                  'fixed_environment_roles':sorted(self.fixed_environment_roles),
                                  'environment_collision_geometry':'triangle_mesh_for_fixed_roles'}
        if self.supporting_hands:
            self.physical_properties.update(controllable_hands=['right',*self.supporting_hands],target_fixture=False,
                supporting_hand_mass_kg={side:float(np.asarray(hand.av.get_masses()).sum())
                                        for side,hand in self.supporting_hands.items()})
        self.ticks=0;self.records=[];self.record_mesh=False;self.restore_count=0
        scene_definition={'objects':initial,'embodiment':self.embodiment,'controller':self.cfg,
            'hand_weight_compensation':('measured_force_and_moment' if self.cfg['controller'].get('hand_gravity_moment_compensation',False)
                else 'measured_force')}
        if self.supporting_hands:
            scene_definition['supporting_hands']={side:dict(object_role=hand.object_role,
                urdf_sha256=hashlib.sha256(hand.args.xhand_urdf.read_bytes()).hexdigest())
                for side,hand in self.supporting_hands.items()}
        self.scene_id=hashlib.sha256(json.dumps(scene_definition,sort_keys=True).encode()).hexdigest()
        self.last_wrench=np.zeros(6);self.command_pose=np.asarray(initial_wrist).copy();self.command_twist=np.zeros(6)
        self.initial_drive_pose=np.asarray(initial_wrist).copy();self.initial_drive_hand=self.hand_command.copy()
        # Use exactly the same physical initialization as local episodes.
        # Constructor defaults and explicit episode initialization differed
        # by micrometres at contact, then diverged during grasp execution.
        # This runs only before t=0; there are no segment-boundary resets.
        self.begin_initial_trial(out,mesh=True)
        self.restore_count=0

    def root_motion(self):
        p,q=self.wrist.get_world_pose()
        # PhysX link linear velocity is at COM; control targets refer to link origin.
        v=np.asarray(self.wrist.get_linear_velocity());w=np.asarray(self.wrist.get_angular_velocity())
        com=np.asarray(self.av.get_coms())[0,0,:3]
        origin_v=v+np.cross(w,-Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).apply(com))
        return np.r_[p,q],np.r_[origin_v,w]

    def _drive(self,pose,hand,twist):
        actual,velocity=self.root_motion();cfg=self.cfg['controller']
        from multipath.execution.wrench_handoff import feedback_wrench,carried_gravity_wrench
        response=feedback_wrench(pose,twist,actual,velocity,cfg)+getattr(self,'wrench_bias',np.zeros(6))
        from multipath.execution.transport_dynamics import hand_gravity_torque
        self.self_gravity_torque=(hand_gravity_torque(self,actual)
            if cfg.get('hand_gravity_moment_compensation',False) else np.zeros(3))
        response[3:]+=self.self_gravity_torque
        if getattr(self,'transport_load',False):response+=carried_gravity_wrench(self)
        force=np.clip(response[:3],-cfg['wrist_max_force'],cfg['wrist_max_force'])
        torque=np.clip(response[3:],-cfg['wrist_max_torque'],cfg['wrist_max_torque'])
        force+=np.array([0,0,9.81])*np.asarray(self.av.get_masses()).sum()
        n=self.av.max_links
        forces=np.zeros((1,n,3),dtype=np.float32);torques=np.zeros_like(forces)
        forces[0,0]=force;torques[0,0]=torque
        self.av.apply_forces_and_torques_at_position(forces,torques,None,np.array([0],dtype=np.uint32),True)
        q=np.asarray(self.robot.get_joint_positions()).copy();q[self.fingers]=hand
        target_velocity=np.zeros_like(q)
        self.hand_command_velocity=(np.clip((hand-self.hand_command)/self.dt,-self.hand_velocity,self.hand_velocity)
            if cfg.get('hand_velocity_feedforward',False) else np.zeros_like(hand))
        target_velocity[self.fingers]=self.hand_command_velocity
        self.robot.apply_action(self.Action(joint_positions=q,joint_velocities=target_velocity))
        self.hand_command=np.array(hand).copy();self.command_pose=np.array(pose).copy();self.command_twist=np.array(twist).copy();self.last_wrench=np.r_[force,torque]
        for support in getattr(self,'supporting_hands',{}).values():
            command=support.pending_command
            if command is None:command=np.r_[support.command_pose,support.hand_command,np.zeros(6)]
            support._drive(command[:7],command[7:19],command[19:25])

    def state(self):
        from multipath.validation.physics import hand_tool_contact
        pose,twist=self.root_motion()
        q=np.asarray(self.robot.get_joint_positions())[self.fingers];dq=np.asarray(self.robot.get_joint_velocities())[self.fingers]
        objects={}
        for role,obj in self.objects.items():
            p,r=obj.get_world_pose();objects[role]={'pose':np.r_[p,r].tolist(),'velocity':np.r_[obj.get_linear_velocity(),obj.get_angular_velocity()].tolist()}
        values=np.r_[pose,twist,q,dq,*[np.r_[v['pose'],v['velocity']] for v in objects.values()]]
        if not np.isfinite(values).all():raise RuntimeError('Non-finite floating PhysX state')
        state={'embodiment':self.embodiment,'wrist_xyz_wxyz':pose.tolist(),'wrist_twist':twist.tolist(),'hand_q':q.tolist(),'hand_dq':dq.tolist(),
            'objects':objects,'hand_object_transform':(np.linalg.inv(pose_matrix(pose))@pose_matrix(objects['tool']['pose'])).tolist(),
            'contact':hand_tool_contact(self.contacts.prim_paths,self.contacts.get_contact_force_matrix(dt=self.dt)),
            'physical_properties':dict(self.physical_properties),
            'interaction_contacts':{'tool_target_contact_steps':self.tool_target_contact_steps,
                                    'tool_target_impulse_n_s':self.tool_target_impulse},
            'scene_id':self.scene_id,'time_s':self.ticks*self.dt,'physically_acquired_state':True,'state_origin':'continuous_PhysX_rollout',
            'controller_wrench_bias':getattr(self,'wrench_bias',np.zeros(6)).tolist(),
            'hand_self_gravity_torque':self.self_gravity_torque.tolist(),
            'command_hand_dq':self.hand_command_velocity.tolist(),
            'command_wrist':self.command_pose.tolist(),'command_twist':self.command_twist.tolist(),'command_hand_q':self.hand_command.tolist()}
        if getattr(self,'supporting_hands',None):
            keys=('wrist_xyz_wxyz','wrist_twist','hand_q','hand_dq','contact','hand_object_transform',
                  'command_wrist','command_twist','command_hand_q','command_hand_dq','controller_wrench_bias')
            state['hands']={'right':dict({key:state[key] for key in keys},object_role='tool')}
            state['hands'].update({side:hand.state() for side,hand in self.supporting_hands.items()})
        return state

    def capture(self,mode,frame):
        self.records.append(dict(self.state(),mode=mode,source_frame=frame,command_wrist=self.command_pose.tolist(),
            command_twist=self.command_twist.tolist(),command_hand_q=self.hand_command.tolist(),applied_wrench=self.last_wrench.tolist()))
        if self.record_mesh:self.recorder.capture()

    def step_control(self,command,mode,frame,*,supporting_commands=None):
        if not np.isclose(frame,(self.ticks+1)*self.dt*self.manifest['fps'],rtol=0,atol=1e-8):
            raise ValueError('Command source frame differs from actual physics clock')
        provider=getattr(self,'supporting_control_provider',None)
        if provider is not None and supporting_commands is None:
            supplied=provider(frame)
            supporting_commands={side:value[0] for side,value in supplied.items()}
            for side,(_,wrench) in supplied.items():self.supporting_hands[side].wrench_bias=np.asarray(wrench)
        provider=getattr(self,'control_wrench_provider',None)
        if provider is not None:self.wrench_bias=np.asarray(provider(frame,command))
        for side,value in (supporting_commands or {}).items():
            self.supporting_hands[side].pending_command=np.asarray(value)
        self._drive(command[:7],command[7:19],command[19:25]);self.world.step(render=False);self.ticks+=1
        self.executed_controls.append({
            'right':np.r_[self.command_pose,self.hand_command,self.command_twist].copy(),
            'right_wrenches':self.wrench_bias.copy(),
            **{side:np.r_[hand.command_pose,hand.hand_command,hand.command_twist].copy()
               for side,hand in getattr(self,'supporting_hands',{}).items()},
            **{side+'_wrenches':hand.wrench_bias.copy()
               for side,hand in getattr(self,'supporting_hands',{}).items()}})
        from multipath.validation.physics import hand_tool_contact
        for side,hand in {'right':self,**getattr(self,'supporting_hands',{})}.items():
            contact=hand_tool_contact(hand.contacts.prim_paths,hand.contacts.get_contact_force_matrix(dt=self.dt))
            self.executed_controls[-1][side+'_native_opposition']=contact['opposition_contact']
        for support in getattr(self,'supporting_hands',{}).values():support.pending_command=None
        if self.tool_target_contacts is not None:
            force=float(np.linalg.norm(self.tool_target_contacts.get_contact_force_matrix(dt=self.dt)))
            if not np.isfinite(force):raise RuntimeError('Non-finite tool-target contact force')
            self.tool_target_contact_steps+=int(force>.01)
            self.tool_target_impulse+=force*self.dt
        if self.ticks%self.tpf==0:self.capture(mode,frame)

    def snapshot(self):
        if self.ticks==getattr(self,'restored_tick',None):return dict(self.restored_state)
        return dict(self.state(),entry_origin='predecessor_rollout',
            source_provenance=self.manifest['provenance'],
            entry_provenance={'trace':str(self.out/'actual_trace.json'),'source_frame':self.ticks*self.dt*self.manifest['fps'],'predecessor':dict(self.lineage)},
            command_wrist=self.command_pose.tolist(),command_twist=self.command_twist.tolist(),command_hand_q=self.hand_command.tolist())

    def restore_development(self,state):
        from multipath.anchors.artifact import reachable_entry
        if not reachable_entry(state) or state['scene_id']!=self.scene_id or state.get('embodiment')!=self.embodiment:
            raise ValueError('Floating development requires a physically acquired floating snapshot')
        # Set root pose only during local development, never during final execution.
        pose=np.asarray(state['wrist_xyz_wxyz']);self.robot.set_world_pose(position=pose[:3],orientation=pose[3:])
        twist=np.asarray(state['wrist_twist']);com=np.asarray(self.av.get_coms())[0,0,:3]
        com_v=twist[:3]+np.cross(twist[3:],Rotation.from_quat(pose[[4,5,6,3]]).apply(com))
        self.robot.set_linear_velocity(com_v);self.robot.set_angular_velocity(twist[3:])
        q=np.asarray(self.robot.get_joint_positions()).copy();dq=np.zeros_like(q)
        q[self.fingers]=state['hand_q'];dq[self.fingers]=state['hand_dq']
        self.robot.set_joint_positions(q);self.robot.set_joint_velocities(dq)
        # Restore a grasp's actual recorded preload as well as q/dq. Replacing
        # a deflected finger's drive target with measured q releases the grip.
        preserve='command_hand_q' in state
        self.wrench_bias=np.asarray(state.get('controller_wrench_bias',np.zeros(6))).copy()
        self.transport_load=False
        drive=q.copy()
        if preserve:drive[self.fingers]=state.get('command_hand_q',state['hand_q'])
        target_velocity=np.zeros_like(dq) if preserve else dq.copy()
        if preserve:target_velocity[self.fingers]=state.get('command_hand_dq',np.zeros(12))
        self.robot.apply_action(self.Action(joint_positions=drive,joint_velocities=target_velocity))
        for role,obj in self.objects.items():
            s=state['objects'][role];obj.set_world_pose(position=np.asarray(s['pose'][:3]),orientation=np.asarray(s['pose'][3:]));obj.set_linear_velocity(np.asarray(s['velocity'][:3]));obj.set_angular_velocity(np.asarray(s['velocity'][3:]))
        for side,support in getattr(self,'supporting_hands',{}).items():
            support.restore_development(state['hands'][side])
        self.ticks=round(state['time_s']/self.dt);self.lineage=state['entry_provenance'];self.restore_count+=1
        self.command_pose=np.asarray(state.get('command_wrist',pose) if preserve else pose).copy()
        self.command_twist=np.asarray(state.get('command_twist',twist) if preserve else twist).copy()
        self.hand_command=drive[self.fingers].copy()
        self.hand_command_velocity=np.asarray(state.get('command_hand_dq',np.zeros(12))).copy()
        interaction=state.get('interaction_contacts',{})
        self.tool_target_contact_steps=interaction.get('tool_target_contact_steps',0)
        self.tool_target_impulse=interaction.get('tool_target_impulse_n_s',0.)
        self.restored_tick=self.ticks
        self.restored_state=dict(state,command_wrist=self.command_pose.tolist(),command_twist=self.command_twist.tolist(),
                                 command_hand_q=self.hand_command.tolist())

    def begin_recording(self,out,mesh=False):
        self.out=Path(out);self.out.mkdir(parents=True,exist_ok=True);self.records=[];self.record_mesh=mesh;self.record_start_tick=self.ticks
        self.executed_controls=[]
        if mesh:self.recorder=ActualMeshRecorder(self.world.stage,self.out)

    def begin_trial(self,out,state,mesh=False):
        self.restore_development(state);self.begin_recording(out,mesh)

    def begin_initial_trial(self,out,mesh=False):
        """Reacquire the initial preparation with fresh solver contact history.

        Used only by local development before source time zero. Merely
        restoring q/dq and object snapshots leaves PhysX warm-start contacts
        from the preceding optimization episode. The final continuous runtime
        still initializes once and never calls this method between segments.
        """
        self.world.reset();self.wrist.initialize()
        for support in getattr(self,'supporting_hands',{}).values():support.initialize()
        view=self.robot._articulation_view;self.av=view._physics_view
        view.set_gains(kps=self.hand_drive_stiffness[None],kds=self.hand_drive_damping[None],joint_indices=self.fingers)
        view.set_max_efforts(self.hand_effort_limits[None],joint_indices=self.fingers)
        pose=self.initial_drive_pose
        self.robot.set_world_pose(position=pose[:3],orientation=pose[3:])
        self.robot.set_linear_velocity(np.zeros(3));self.robot.set_angular_velocity(np.zeros(3))
        q=np.zeros(self.robot.num_dof);q[self.fingers]=self.initial_drive_hand
        self.robot.set_joint_positions(q);self.robot.set_joint_velocities(np.zeros_like(q))
        self.hand_command=self.initial_drive_hand.copy();self.hand_command_velocity=np.zeros(12)
        for role,obj in self.objects.items():
            initial=self.manifest['initial_objects'][role]['pose']
            obj.set_world_pose(position=np.asarray(initial[:3]),orientation=np.asarray(initial[3:]))
            obj.set_linear_velocity(np.zeros(3));obj.set_angular_velocity(np.zeros(3))
        self.wrench_bias=np.zeros(6);self.transport_load=False
        for _ in range(120):
            self._drive(pose,self.initial_drive_hand,np.zeros(6));self.world.step(render=False)
        self.ticks=0;self.restored_tick=None;self.restore_count+=1;self.lineage={'initial_scene':self.scene_id}
        self.tool_target_contact_steps=0;self.tool_target_impulse=0.
        self.begin_recording(out,mesh);self.capture('INITIAL',0)

    def save(self):
        if self.record_mesh:self.recorder.save()
        if self.executed_controls:
            np.savez_compressed(self.out/'executed_controls.npz',
                **{key:np.asarray([step[key] for step in self.executed_controls])
                   for key in self.executed_controls[0]})
        (self.out/'actual_trace.json').write_text(json.dumps(self.records))
        (self.out/'pipeline_result.json').write_text(json.dumps({'embodiment':self.embodiment,'source_fps':self.manifest['fps'],
            'table_z':self.table['pose'][2]+.02,'source':'oracle','actual_duration_s':(self.ticks-self.record_start_tick)*self.dt,
            'task_success':False,'validation_scope':'local_development_actual_state'},indent=2))

    def check_transition(self,pose,contact,expected_object=None,canonical=None):
        from multipath.validation.physics import core_pose_errors,object_tracking_score
        s=self.state();actual=np.asarray(s['wrist_xyz_wxyz']);pose=np.asarray(pose)
        error={'position_m':float(np.linalg.norm(actual[:3]-pose[:3])),
               'rotation_rad':float((Rotation.from_quat(actual[[4,5,6,3]])*Rotation.from_quat(pose[[4,5,6,3]]).inv()).magnitude())}
        # A source target is not a physically validated canonical entry.
        # Wrist tracking is diagnostic here; actual contact/object state and
        # canonical compatibility (when supplied) decide the transition.
        ok=s['contact']['mode']==contact
        report={'wrist_error':error,'wrist_tracking_diagnostic_only':canonical is None,
            'actual_contact':s['contact'],'actual_wrist_twist':s['wrist_twist']}
        if expected_object is not None:
            report['object_boundary_error']=core_pose_errors(s,expected_object);ok=ok and object_tracking_score(report['object_boundary_error'])['feasible']
        if canonical is not None:
            from multipath.anchors.compatibility import boundary_errors,compatible
            report['canonical_errors']=boundary_errors(s,canonical);ok=ok and compatible(report['canonical_errors'])
        report['success']=bool(ok);return report
