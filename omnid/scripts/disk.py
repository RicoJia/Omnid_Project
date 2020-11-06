#!/usr/bin/env python3

import pybullet as p
import numpy as np

class Disk:

  def __init__(self, urdf_path, spawn_position=[0.0, 0.0, 1.0], urdf_name = "disk.urdf"):
    urdf_full_path = urdf_path + "/" + urdf_name
    self.reset(urdf_full_path, spawn_position)

  def reset(self, urdf_full_path, spawn_position):
    self.model_unique_id = p.loadURDF(urdf_full_path, basePosition=spawn_position)
    self.spawn_position = spawn_position

  def getObjectInfo(self):
    """ return information such as spawn position, disk radius in a dictionary"""
    return {"spawn_position": self.spawn_position
            }

  def getEndEffectorLinkID(self):
    """ return the end effector link ID """
    return -1



 