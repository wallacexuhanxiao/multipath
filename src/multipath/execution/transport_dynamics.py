"""Rigid-grasp feedforward for a planned floating-wrist trajectory.

Mass properties come from the actual PhysX assembly. The rigid-grasp model
only supplies a control input; measured contacts and collisions still decide
whether the executed bridge is valid.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def hand_gravity_torque(scene,root_pose):
    """Cancel the hand links' own gravity moment about the driven root COM."""
    p,q=scene.contacts.get_world_poses();com,_=scene.contacts.get_coms()
    mass=np.asarray(scene.contacts.get_masses()).reshape(-1)
    center=np.asarray(p).reshape(-1,3)+Rotation.from_quat(np.asarray(q).reshape(-1,4)[:,[1,2,3,0]]).apply(np.asarray(com).reshape(-1,3))
    root_com=np.asarray(scene.av.get_coms())[0,0,:3]
    origin=np.asarray(root_pose)[:3]+Rotation.from_quat(np.asarray(root_pose)[[4,5,6,3]]).apply(root_com)
    force=np.zeros((len(mass),3));force[:,2]=mass*9.81
    return np.cross(center-origin,force).sum(0)


class TransportDynamics:
    def __init__(self, scene, carry):
        root, _ = scene.root_motion()
        root_rotation = Rotation.from_quat(root[[4, 5, 6, 3]])
        masses, centers, inertias = [], [], []
        views = [scene.contacts]
        if carry:
            views.append(scene.objects['tool']._rigid_prim_view)
        for view in views:
            p, q = view.get_world_poses()
            com, com_q = view.get_coms()
            mass = np.asarray(view.get_masses()).reshape(-1)
            body_rotation = Rotation.from_quat(np.asarray(q).reshape(-1, 4)[:, [1, 2, 3, 0]])
            center = np.asarray(p).reshape(-1, 3) + body_rotation.apply(np.asarray(com).reshape(-1, 3))
            if view is scene.contacts:
                self.hand_center=np.sum(mass[:,None]*root_rotation.inv().apply(center-root[:3]),axis=0)/mass.sum()
            principal = body_rotation * Rotation.from_quat(np.asarray(com_q).reshape(-1, 4)[:, [1, 2, 3, 0]])
            orientation = (root_rotation.inv() * principal).as_matrix()
            inertia = np.asarray(view.get_inertias()).reshape(-1, 3, 3)
            masses.extend(mass)
            centers.extend(root_rotation.inv().apply(center-root[:3]))
            inertias.extend(orientation @ inertia @ orientation.transpose(0, 2, 1))
        masses = np.asarray(masses); centers = np.asarray(centers)
        self.mass = float(masses.sum())
        self.center = np.sum(masses[:, None]*centers, axis=0)/self.mass
        parallel = np.sum(centers*centers, axis=1)[:, None, None]*np.eye(3)-centers[:, :, None]*centers[:, None, :]
        self.inertia = np.sum(np.asarray(inertias)+masses[:, None, None]*parallel, axis=0)
        self.hand_mass = float(np.asarray(scene.av.get_masses()).sum())
        self.hand_gravity_moment_compensation=scene.cfg['controller'].get('hand_gravity_moment_compensation',False)
        self.root_com = np.asarray(scene.av.get_coms())[0, 0, :3].copy()
        if not np.isfinite(np.r_[self.mass, self.center, self.inertia.ravel()]).all() or self.mass <= 0:
            raise ValueError('Invalid measured transport mass properties')

    def wrench(self, pose, twist, acceleration):
        rotation = Rotation.from_quat(np.asarray(pose)[[4, 5, 6, 3]]).as_matrix()
        center = rotation @ self.center
        inertia = rotation @ self.inertia @ rotation.T
        omega = np.asarray(twist)[3:]; alpha = np.asarray(acceleration)[3:]
        nongravity = np.asarray(acceleration)[:3]+np.array([0., 0., 9.81])
        force = self.mass*(nongravity+np.cross(alpha, center)+np.cross(omega, np.cross(omega, center)))
        torque = inertia @ alpha+np.cross(omega, inertia @ omega)+np.cross(center, self.mass*nongravity)
        torque -= np.cross(rotation @ self.root_com, force)
        # Subtract only gravity terms actually applied by the controller.
        # Controller variants are explicit, so a validated drive trajectory
        # cannot silently acquire a different gravity-compensation model.
        force -= np.array([0., 0., self.hand_mass*9.81])
        if self.hand_gravity_moment_compensation:
            torque -= np.cross(rotation@(self.hand_center-self.root_com),np.array([0.,0.,self.hand_mass*9.81]))
        return np.r_[force, torque]

    def commands(self, scene, start, commands):
        previous = np.vstack((start['wrist_twist'], commands[:-1, 19:25]))
        acceleration = (commands[:, 19:25]-previous)/scene.dt
        return np.asarray([self.wrench(c[:7], c[19:25], a) for c, a in zip(commands, acceleration)])

    def report(self):
        return dict(model='measured_rigid_grasp_Newton_Euler', mass_kg=self.mass,
                    hand_mass_kg=self.hand_mass, center_root_m=self.center.tolist(),
                    inertia_root_kg_m2=self.inertia.tolist(), physical_validation_required=True)
