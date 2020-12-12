#!/usr/bin/env python3

import pybullet as p
import numpy as np
import rospy
from sensor_msgs.msg import JointState

class Disk:
  def __init__(self, urdf_path, spawn_position=[0.0, 0.0, 1.0], urdf_name = "disk.urdf"):
    urdf_full_path = urdf_path + "/" + urdf_name
    #pos will be [x,y,z], ori will be [roll, pitch, yaw]
    self.joint_names = rospy.get_param("omnid_simulator/object_platform_joint_info").keys()
    self.reset(urdf_full_path, spawn_position)
    self.object_thickness = rospy.get_param("object_thickness") #thickness of the end effector platform
    self.z_correction= rospy.get_param("base_offset")    #The base was offseted in the URDF of the robot. Change this value if URDF has changed.

  def reset(self, urdf_full_path, spawn_position):
    self.model_unique_id = p.loadURDF(urdf_full_path, basePosition=spawn_position)
    self.spawn_position = spawn_position

  def getObjectInfo(self):
    """ return information such as spawn position, disk radius in a dictionary"""
    return {"spawn_position": self.spawn_position}

  def getEndEffectorLinkID(self):
    """ return the end effector link ID """
    return -1

  def getJointStates(self):
    """ return the link states as a joint_state_msg"""
    joint_state_msg = JointState()
    joint_state_msg.name = self.joint_names
    pos, ori = p.getBasePositionAndOrientation(self.model_unique_id)
    pos = [pos[0] - self.spawn_position[0], pos[1] - self.spawn_position[1], pos[2] - self.z_correction]    #x, y,z is defined as the relative pose to base.
    lin_vel, an_vel = p.getBaseVelocity(self.model_unique_id)
    rpy = p.getEulerFromQuaternion(ori)
    joint_state_msg.position += pos
    joint_state_msg.position += rpy
    joint_state_msg.velocity += lin_vel
    joint_state_msg.velocity += an_vel  #an_vel is already Cartesian, but not sure if it's z,y,x
    joint_state_msg.effort += [0,0,0,0,0,0]
    return joint_state_msg




