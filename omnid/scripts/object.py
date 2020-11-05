#!/usr/bin/env python3

import pybullet as p
import numpy as np

class Spring_Model:

  def __init__(self, model_unique_id, 
               child_link_id, force_axis_c, moment_arm_c, force_pos_c,
               parent_link_id, force_axis_p, moment_arm_p, force_pos_p,
               primary_joint_id, secondary_joint_id, torque_k=0.1):
    """
        In URDF, there might be two spring joints: one is attached to the end of the parent link (joint angle phi), one is attached to the beginning of the child link (joint angle theta).
        Phi is defined as the angle between the parent link (like a base)and a dummy spring link, theta is defined as the angle between the parent link and the child link (like an arm)
    :param model_unique_id: int, the model id of an omnid robot
    :param child_link_id: int, actual link spring force exerted on, like an arm.
    :param force_axis_c: array-like, unit vector of force in child link frame, when (phi - theta) is positive. [0,0,1]
    :param moment_arm_c: float, length of the child link origin to the spring
    :param parent_link_id: int, actual link spring force exerted on, like base_link
    :param force_axis_p: unit vector of force in parent link frame, when (phi - theta) is positive [0, 0, 1]
    :param moment_arm_p: float, length of the parent link origin to the spring
    :param primary_joint_id: joint connecting the base and the lower_arm_attached. Note that on Omnid, this is not an actuated joint, but a reference joint.
    :param secondary_joint_id: joint connecting the base and the lower arm, on Omnid this is an actuated joint.
    :param torque_k: torsion spring constant
    """
    self.model_unique_id = model_unique_id
    self.child_link_id = child_link_id
    self.scaled_force_axis_c = 1.0/moment_arm_c * np.array(force_axis_c)
    self.parent_link_id = parent_link_id
    self.scaled_force_axis_p = 1.0/moment_arm_p * np.array(force_axis_p)
    self.primary_joint_id = primary_joint_id
    self.secondary_joint_id = secondary_joint_id
    self.force_pos_c = force_pos_c
    self.force_pos_p = force_pos_p
    self.torque_k = torque_k
    self.reset()

  def reset(self):
    #enable the actuated joint sensor on the secondary joint
    p.enableJointForceTorqueSensor(bodyUniqueId = self.model_unique_id, jointIndex = self.secondary_joint_id, enableSensor = True)

  def apply_spring_torque(self):
    """
    The main API function other application should call to execute apply the corresponding torque.
    On omnid, the secondary joint (joint connecting the base and the lower arm) is the actuated joint
    """
    #This is essential, as this disables the default motor imposed by bullet. These default motors prevent overspeeding, and functions as
    #friction drives.
    p.setJointMotorControl2(self.model_unique_id, self.secondary_joint_id,
                              controlMode=p.VELOCITY_CONTROL, force=0)
    spring_angle = self.get_joint_angle()
    torque = spring_angle * self.torque_k

    #apply external force
    self.apply_external_force(force= torque * self.scaled_force_axis_c, link_index=self.child_link_id, position=self.force_pos_c)
    self.apply_external_force(force= torque * self.scaled_force_axis_p, link_index=self.parent_link_id, position=self.force_pos_p)
    # print("torque: ", torque)

  def get_joint_angle(self):
    """
    :return: angle between phi and theta (primary_joint - secondary_joint)
    """
    primary_joint_info = p.getJointState(bodyUniqueId = self.model_unique_id, jointIndex=self.primary_joint_id)
    secondary_joint_info = p.getJointState(bodyUniqueId = self.model_unique_id, jointIndex=self.secondary_joint_id)
    phi = primary_joint_info[0]
    theta = secondary_joint_info[0]
    # print("theta: ", theta, "phi: ", phi)
    return phi - theta

  def apply_external_force(self, force, link_index, position=[0,0,0]):
    """
    Apply the specified external force on the specified position on the body / link.
    Args:
        link_index (int): unique link id. If -1, it will be the base.
        force (np.array[float[3]]): external force to be applied.
        position (np.array[float[3]], None): position on the link where the force is applied. See `flags` for
        coordinate systems. If None, it is the center of mass of the body (or the link if specified).
        frame (int): Specify the coordinate system of force/position: either `pybullet.WORL   D_FRAME` (=2) for
        Cartesian world coordinates or `pybullet.LINK_FRAME` (=1) for local link coordinates.
    """
    p.applyExternalForce(
      objectUniqueId=self.model_unique_id, linkIndex=link_index,
      forceObj=force, posObj=position, flags=p.LINK_FRAME)
    # print("link_id: ", link_index, " | force: ", force)
