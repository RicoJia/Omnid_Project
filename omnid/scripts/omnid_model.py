#!/usr/bin/env python3

import pybullet as p
import numpy as np
from torsion_spring_omnid import Spring_Model
import rospy
from sensor_msgs.msg import JointState

class Omnid_Model:

  def __init__(self, urdfRootPath='', use_spring=False, test_with_end_effector_xyz=False, after_spring_actuated=False, urdf_name="delta_robot_pybullet.urdf"):
    self.urdfRootPath = urdfRootPath
    self.use_spring = use_spring
    self.reset(test_with_end_effector_xyz, after_spring_actuated, urdf_name)

  def reset(self, test_with_end_effector_xyz, after_spring_actuated, urdf_name):
    self.buildParamLists(test_with_end_effector_xyz, after_spring_actuated)
    self.model_unique_id = p.loadURDF(self.urdfRootPath + "/" + urdf_name , basePosition=[0,0,self.base_offset], useFixedBase=True)
    self.buildLookups()
    self.resetLinkFrictions(lateral_friction_coefficient=0)
    self.resetJointsAndMotors()
    self.buildClosedChains()
    if self.use_spring:
        self.addSpringJoints()


  def buildParamLists(self, test_with_end_effector_xyz, after_spring_actuated):
      """
      Read in physics params from launch file. Here we assume: there are multiple legs, each leg has exactly the same joint names and link names
      the only varying part is its leg id. 
      Then, we build Link and joint name lookups (dictionary) for various uses: 
        pre_spring joint names, after_spring_joint_names, spring_parent_link_name, spring_child_link_name, end-effector names
      """
      #TODO rospy.get_param("~")
      self.base_offset= rospy.get_param("~base_offset")    #The base was offseted in the URDF of the robot. Change this value if URDF has changed.
      self.leg_num = rospy.get_param("~leg_num")

      self.kp = rospy.get_param("~kp")
      self.kd = rospy.get_param("~kd")
      self.max_motor_force = rospy.get_param("~max_motor_force")      

      self.end_effector_thickness = rospy.get_param("~end_effector_thickness") #thickness of the end effector platform
      self.end_effector_radius = rospy.get_param("~end_effector_radius")
      self.leg_pos_on_end_effector = rospy.get_param("~leg_pos_on_end_effector")
      self.end_effector_name = rospy.get_param("~end_effector_name")

      spring_parent_link_name = rospy.get_param("~spring_parent_link_name")
      spring_child_link_name = rospy.get_param("~spring_child_link_name")
      upper_leg_name = rospy.get_param("~upper_leg_name")
      self.upper_leg_length = rospy.get_param("~upper_leg_length")

      pre_spring_joint_info = rospy.get_param("~pre_spring_joint_info")
      after_spring_joint_info = rospy.get_param("~after_spring_joint_info")
      self.torque_k = rospy.get_param("~torque_k")

      self.force_axis_c = rospy.get_param("~force_axis_c")
      self.moment_arm_c = rospy.get_param("~lower_arm_length")/2.0
      self.force_pos_c = rospy.get_param("~force_pos_c")
      self.force_axis_p = rospy.get_param("~force_axis_p")
      self.moment_arm_p = rospy.get_param("~pre_spring_radius")
      self.force_pos_p = rospy.get_param("~force_pos_p")
      self.end_effector_initial_xyz = rospy.get_param("~end_effector_initial_xyz")


      #spring joints and links
      self.pre_spring_joint_vals = {}
      self.after_spring_joint_vals = {}
      self.spring_parent_link_names = {}
      self.spring_child_link_names = {}
      self.upper_leg_names = {}
      for leg_id in range(1, self.leg_num + 1):
        pre_spring_joint_name = list(pre_spring_joint_info.keys())[0]
        pre_spring_joint_val = list(pre_spring_joint_info.values())[0]
        self.pre_spring_joint_vals[pre_spring_joint_name + "_" + str(leg_id)] = pre_spring_joint_val
        after_spring_joint_name = list(after_spring_joint_info.keys())[0]
        after_spring_joint_val = list(after_spring_joint_info.values())[0]
        self.after_spring_joint_vals[after_spring_joint_name + "_" + str(leg_id)] = after_spring_joint_val
        self.spring_parent_link_names[leg_id] = spring_parent_link_name + "_" + str(leg_id)
        self.spring_child_link_names[leg_id] =  spring_child_link_name + "_" +  str(leg_id)
        self.upper_leg_names[leg_id] = upper_leg_name + "_" + str(leg_id)

      # joints to be initialized
      self.init_joint_values = {**self.pre_spring_joint_vals, **self.after_spring_joint_vals, **self.end_effector_initial_xyz}
      self.motorDict = {**self.pre_spring_joint_vals}

      #if we are in a test mode, we're going to overwrite some dicts
      if test_with_end_effector_xyz:
          self.motorDict = {**self.motorDict, **self.end_effector_initial_xyz}
      if after_spring_actuated:
          self.motorDict = {**self.after_spring_joint_vals}

  def buildLookups(self):
    """
    Build following look ups: 
    1. jointNameToId, linkNameToID (both dictionary). 
    Note that since each link and its parent joint has the
    same ID, you can use this to get link_id as well, except that you will have to access base frame by link_id = -1.
    A very important assumption here is in your URDF, you first have a world link, then have a base_link
    """
    # jointNameToId, linkNameToID
    nJoints = p.getNumJoints(self.model_unique_id)
    self.jointNameToId = {}
    self.linkNameToID={}
    for i in range(nJoints):
      jointInfo = p.getJointInfo(self.model_unique_id, i)
      self.jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
      self.linkNameToID[jointInfo[12].decode('UTF-8')] = jointInfo[0]

  def resetLinkFrictions(self, lateral_friction_coefficient):
      """
      Reset all links friction
      """
      for id in self.linkNameToID.values():
          p.changeDynamics(bodyUniqueId=self.model_unique_id,
                           linkIndex=id,
                           jointLowerLimit = -1000,
                           jointUpperLimit = 1000,
                           jointLimitForce = 0,
                           lateralFriction=lateral_friction_coefficient,
                           spinningFriction=0.0,
                           rollingFriction=0.0,
                           linearDamping = 0.0,
                           angularDamping = 0.0,
                           jointDamping = 0.0,
                           contactStiffness=0.0,
                           contactDamping=0.0,
                           maxJointVelocity=10000
                           )
          
  def resetJointsAndMotors(self):
    """
    We do two things here:
      1. Look up your URDF and set the desired joint angles here.
      2. Motor Control Set up: 1. Specify which joints need to be controlled by motors. 2. specify motor control parameters
    """
    self.maxPoint2PointForce = 5000000

    #disable friction in all joints
    for joint_id in self.jointNameToId.values():
        p.setJointMotorControl2(self.model_unique_id, joint_id,
                                controlMode=p.VELOCITY_CONTROL, force=0)

    #All joint values to be initialized
    for joint_name in self.init_joint_values:
      p.resetJointState(bodyUniqueId = self.model_unique_id, jointIndex=self.jointNameToId[joint_name], targetValue=self.init_joint_values[joint_name])


  def buildClosedChains(self):
    """
    Connect links to joints to build closed chain robots, since URDF does not support chain robots.
    """
    joint_axis = [0,0,0]
    for leg_id in range(1, self.leg_num + 1):
        upper_leg_name = self.upper_leg_names[leg_id]
        x = self.end_effector_radius * np.cos(self.leg_pos_on_end_effector[upper_leg_name])
        y = self.end_effector_radius * np.sin(self.leg_pos_on_end_effector[upper_leg_name])
        parent_frame_pos = np.array([ x, y, -self.end_effector_thickness/2.0])  # Cartesian coordnates on the platform, r_platform = 0.062
        child_frame_pos = [self.upper_leg_length/2.0, 0.0, 0.0]   # L_upper = 0.368/2.0
        new_joint_id = p.createConstraint(self.model_unique_id, self.linkNameToID[self.end_effector_name],
                                          self.model_unique_id, self.linkNameToID[upper_leg_name],
                                          p.JOINT_POINT2POINT, joint_axis, parent_frame_pos, child_frame_pos)
        p.changeConstraint(new_joint_id, maxForce=self.maxPoint2PointForce)

  def addSpringJoints(self):
      """
      Embed springs into omnid's leg
      """
      self.spring_models = {}
      for leg_id in range(1, self.leg_num + 1):
          child_link_id = self.linkNameToID[ self.spring_child_link_names[leg_id]]
          parent_link_id = self.linkNameToID[ self.spring_parent_link_names[leg_id]]
          primary_joint_id = self.jointNameToId[ list(self.pre_spring_joint_vals.keys())[leg_id - 1] ]
          secondary_joint_id = self.jointNameToId[ list(self.after_spring_joint_vals.keys())[leg_id - 1] ]

          self.spring_models[leg_id] = Spring_Model(self.model_unique_id,
                                                    child_link_id, self.force_axis_c, self.moment_arm_c, self.force_pos_c,
                                                    parent_link_id, self.force_axis_p, self.moment_arm_p, self.force_pos_p,
                                                    primary_joint_id,secondary_joint_id,
                                                    self.torque_k)
          
          
########################## Helpers ##########################
  def setMotorValueByName(self, motorName, desiredValue):
    """
    Joint Position Control using PyBullet's Default joint angle control
    :param motorName: string, motor name
    :param desiredValue: float, angle value
    """
    motorId=self.jointNameToId[motorName]
    p.setJointMotorControl2(bodyIndex=self.model_unique_id,
                          jointIndex=motorId,
                          controlMode=p.POSITION_CONTROL,
                          targetPosition=desiredValue,
                          positionGain=self.kp,
                          velocityGain=self.kd,
                          force=self.max_motor_force)
      
  def executeAllMotorPosCommands(self):
    """
    This is a helper function that executes all motor position commands in self.motorDict
    """
    if self.use_spring:
        for spring_id in range(1, self.leg_num + 1):
            self.spring_models[spring_id].apply_spring_torque()

    for name, value in self.motorDict.items():
      self.setMotorValueByName(name, value)

  def returnJointStateMsg(self):
      """
      Publish joint states. Note that effort is the motor torque applied during the last stepSimulation.
      :return:
      """
      joint_state_msg = JointState()
      for joint_name in self.jointNameToId.keys():
          joint_state = p.getJointState(bodyUniqueId = self.model_unique_id, jointIndex=self.jointNameToId[joint_name])
          joint_state_msg.name.append(joint_name)
          joint_state_msg.position.append(joint_state[0])
          joint_state_msg.velocity.append(joint_state[1])
          joint_state_msg.effort.append(joint_state[3])
      return joint_state_msg

  def updateJointStates(self, name_ls, position_ls):
      """
      The core function to update joint states.  If a joint name is not for an actuated joint, it will be skipped without notice.
      :param name_ls: (list-like) names of joints to be updated.
      :param position_ls: (list-like) new positions of joints
      """
      for i in range(len(name_ls)):
          if name_ls[i] in self.motorDict:
              self.motorDict[name_ls[i]] = position_ls[i]




