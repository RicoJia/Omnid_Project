#!/usr/bin/env python3

import pybullet as p
import numpy as np

class Spring_Model:

  def __init__(self, urdfRootPath='', joint_val=0.0, joint_name="spring_joint", torque_k=0.1):
    """
    In URDF, the spring joint should be attached at the ends of two links, and its frame is aligned with the two links,  .
    :param urdfRootPath: path to the URDF directory
    :param joint_val:  initial joint angle (radian) between the child link and the parent link to initialize the spring's potential energy. If set to 0, the spring will start at the pose specified in URDF.
    :param joint_name: name of the spring in URDF. By default, it's "spring_joint"
    """
    self.urdfRootPath = urdfRootPath
    self.reset(torque_k=torque_k, joint_val=joint_val, joint_name=joint_name)

  def reset(self, torque_k=0.1, joint_val=0.0, joint_name="spring_joint"):
    self.model_body_unique_id = p.loadURDF("%s/spring/spring.urdf" % self.urdfRootPath, basePosition=[0,0,0.5], useFixedBase=1)  #TODO
    self.joint_name = joint_name
    self.buildJointLinkLookups()
    self.enable_joint_sensors()
    self.torque_k = torque_k
    #TODO to delete
    spring_joint_id = self.jointNameToId["spring_joint"]
    p.resetJointState(bodyUniqueId = self.model_body_unique_id, jointIndex=spring_joint_id, targetValue=joint_val)
    self.resetAllFrictions(friction_coefficient=0)

  def buildJointLinkLookups(self):
    """
    builds a dictionary {joint_name: joint_id}. Note that since each link and its parent joint has the
    same ID, you can use this to get link_id as well, except that you will have to access base frame by link_id = -1.
    """
    nJoints = p.getNumJoints(self.model_body_unique_id)
    self.jointNameToId = {}
    for i in range(nJoints):
      jointInfo = p.getJointInfo(self.model_body_unique_id, i)
      self.jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
      #TODO
    self.linkIDs = list(self.jointNameToId.values())
    self.linkIDs.append(-1) #base_link id
    print(self.linkIDs)
    
  def resetAllFrictions(self, friction_coefficient):
    for id in self.linkIDs:
      p.changeDynamics(bodyUniqueId=self.model_body_unique_id,
                       linkIndex=id,
                       jointLowerLimit = -1000,
                       jointUpperLimit = 1000,
                       jointLimitForce = 0,
                       lateralFriction=0.0,
                       spinningFriction=0.0,
                       rollingFriction=0.0,
                       linearDamping = 0.0,
                       angularDamping = 0.0,
                       jointDamping = 0.0,
                       contactStiffness=0.0,
                       contactDamping=0.0,
                       maxJointVelocity=10000
                       )
      # for joint_id in self.jointNameToId.values():  #This is essential, as this disables the default motor imposed by bullet. These default motors prevent overspeeding, and functions as
      #     #friction drives.
      #     p.setJointMotorControl2(self.model_body_unique_id, joint_id,
      #                             controlMode=p.VELOCITY_CONTROL, force=0)
  def apply_torque_spring(self):

      for joint_id in self.jointNameToId.values():  #This is essential, as this disables the default motor imposed by bullet. These default motors prevent overspeeding, and functions as
      #friction drives.
          p.setJointMotorControl2(self.model_body_unique_id, joint_id,
                          controlMode=p.VELOCITY_CONTROL, force=0)

      # spring_joint_down_id = self.jointNameToId["spring_joint_down"]
      spring_joint_id = self.jointNameToId[self.joint_name]
      base_id = -1
      end_effector_id = list(self.jointNameToId.values())[-1]
      joint_states_dict = self.get_joint_state(joint_index=spring_joint_id)
      joint_info_dict = self.get_joint_info(joint_index=spring_joint_id)
      torque = joint_states_dict["jointPosition"] * self.torque_k

      #apply external force
      dist_down = 1.05   #moment arm to the downstream arm
      dist_up = 1.05    #moment arm to the upstream arm (fixed joint)
      force_axis = np.array([0.0, -1.0, 0.0])
      self.apply_external_force(force= torque/dist_down*force_axis, link_index=end_effector_id)
      self.apply_external_force(force= 1.0 * torque/dist_up*force_axis, link_index=base_id)
      # print("joint: ", joint_states_dict["jointPosition"], " | force: ", torque/dist_down)

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
          objectUniqueId=self.model_body_unique_id, linkIndex=link_index,
          forceObj=force, posObj=position, flags=p.LINK_FRAME)

  def non_zero_velocity_control(self):
      """
      Test function to be removed - test if mode switching is possible
      """
      # for joint_id in self.jointNameToId.values():  #This is essential, as this disables the default motor imposed by bullet. These default motors prevent overspeeding, and functions as
      # #friction drives.
      for joint_id in self.jointNameToId.values():  #This is essential, as this disables the default motor imposed by bullet. These default motors prevent overspeeding, and functions as
          #friction drives.
          p.setJointMotorControl2(self.model_body_unique_id, joint_id,
                                  targetPosition= 2.0,
                                  controlMode=p.POSITION_CONTROL, force=100)

  def enable_joint_sensors(self):
      for joint_index in self.jointNameToId.values():
          p.enableJointForceTorqueSensor(bodyUniqueId = self.model_body_unique_id, jointIndex = joint_index, enableSensor = True)

  def get_joint_state(self, joint_index=None, joint_name=None):
      """
      return a dictionary of joint information. You have to provide one of joint_index or joint_name. If both provided, joint_name will be used to find the joint index
      :param link_index: (int) unique link id. if -1, it will be the base
      :return: dictionary of joint information:
        jointPosition (float)
        jointVelocity(float)
        jointReactionForces(list of 6 floats)
        appliedJointMotorTorque (float): These are the joint reaction forces, if a torque sensor is enabled for this joint it is
        [Fx, Fy, Fz, Mx, My, Mz]. Without torque sensor, it is [0,0,0,0,0,0].
      """
      if joint_name is not None:
          joint_index = self.jointNameToId[joint_name]
      info = p.getJointState(bodyUniqueId = self.model_body_unique_id, jointIndex=joint_index)
      return {"jointPosition":info[0], "jointVelocity":info[1], "jointReactionForces":info[2], "appliedJointMotorTorque":info[3]}

  def get_link_state(self, link_index=None):
      """
      :param link_index: (int) unique link id. if -1, it will be the base
      :return: dictionary of joint information:
        link_world_position (float3)
        link_world_orientation(float4) - this is quaternion
        appliedJointMotorTorque (float): These are the joint reaction forces, if a torque sensor is enabled for this joint it is
        [Fx, Fy, Fz, Mx, My, Mz]. Without torque sensor, it is [0,0,0,0,0,0].
      """
      info = p.getLinkState(bodyUniqueId = self.model_body_unique_id, linkIndex=link_index)
      return {"link_world_position":info[0], "link_world_orientation":info[1]}

  def get_joint_info(self, joint_index):
      info = p.getJointInfo(bodyUniqueId = self.model_body_unique_id, jointIndex=joint_index)
      # return {"jointDamping":info[7],  "jointFriction":info[8], "jointLowerLimit":info[9],
      #         "jointUpperLimit":info[10], "jointMaxForce":info[11], "jointMaxVelocity":info[12]}
      #TODO
      return info




