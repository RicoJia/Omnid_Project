#!/usr/bin/env python3

import pybullet as p
import numpy as np

class Omnid_Model:

  def __init__(self, urdfRootPath=''):
    self.urdfRootPath = urdfRootPath
    self.reset()
      
  # def buildJointNameToIdDict(self):
  #   nJoints = p.getNumJoints(self.diff_drive)
  #   self.jointNameToId = {}
  #   for i in range(nJoints):
  #     jointInfo = p.getJointInfo(self.diff_drive, i)
  #     self.jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
  #   self.resetPose()
  #   for i in range(100):
  #     p.stepSimulation()
  #
  # def buildMotorIdList(self):
  #   self.motorIdList.append(self.jointNameToId["right_wheel_axle"])
  #   self.motorIdList.append(self.jointNameToId["left_wheel_axle"])

  def reset(self):
    tex = p.loadTexture("%s/torus/uvmap.png" % self.urdfRootPath)

    # self.spring = p.loadURDF("%s/torus/torus_deform.urdf" % self.urdfRootPath, [0,1,0.5], [0.8509035, 0, 0, 0.525322], flags=p.URDF_USE_SELF_COLLISION)
    # p.changeVisualShape(self.spring, -1, rgbaColor=[1,1,1,1], textureUniqueId=tex, flags=0)

    # self.spring = p.loadSoftBody("%s/torus/torus_textured.obj" % self.urdfRootPath, basePosition = [0,0,1.0], baseOrientation = [0.8509035, 0, 0, 0.525322], mass = 3, useNeoHookean = 1, NeoHookeanMu = 180, NeoHookeanLambda = 600, NeoHookeanDamping = 0.01,
    #                          springElasticStiffness=0.1, springDampingStiffness=1,
    #                          collisionMargin = 0.006, useSelfCollision = 1, frictionCoeff = 0.5, repulsionStiffness = 800)


    # p.createSoftBodyAnchor(self.spring  ,24,-1,-1)
    # p.createSoftBodyAnchor(self.spring  ,20,-1,-1)
    #
    # self.cloth = p.loadURDF("%s/cloth/torus_deform.urdf" % self.urdfRootPath, [0,1,0.5], [0.8509035, 0, 0, 0.525322], flags=p.URDF_USE_SELF_COLLISION)
    # %s/cloth/cloth_z_up.obj
    # %s/torus/torus_textured.obj
    # %s/cube/cube.obj
    # self.cloth = p.loadSoftBody("%s/cube/cube.obj" % self.urdfRootPath, basePosition = [0,0,2], scale = 0.1, mass = 1.,
    #                          useNeoHookean = 0, useBendingSprings=0, useMassSpring=1, springElasticStiffness=400, springDampingStiffness=.1,
    #                          springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = .5, useFaceContact=0)
    # p.changeVisualShape(self.cloth, -1, rgbaColor=[1,1,1,1], textureUniqueId=tex, flags=0)
    # p.createSoftBodyAnchor(self.cloth  ,24,-1,-1)
    # # p.createSoftBodyAnchor(self.cloth  ,20,-1,-1)
    # p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)

    self.diff_drive = p.loadURDF("%s/diff_drive.urdf" % self.urdfRootPath, 0, 0, 0)

    # self.kp = 1
    # self.kd = 0.1
    # self.maxForce = 35
    # self.nMotors = 2
    # self.motorIdList = []
    # self.motorDir = [-1, -1]
    # self.buildJointNameToIdDict()
    # self.buildMotorIdList()

  # def setMotorAngleById(self, motorId, desiredAngle):
  #   p.setJointMotorControl2(bodyIndex=self.diff_drive,
  #                           jointIndex=motorId,
  #                           controlMode=p.POSITION_CONTROL,
  #                           targetPosition=desiredAngle,
  #                           positionGain=self.kp,
  #                           velocityGain=self.kd,
  #                           force=self.maxForce)
  #
  # def setMotorAngleByName(self, motorName, desiredAngle):
  #   self.setMotorAngleById(self.jointNameToId[motorName], desiredAngle)
  #
  # def resetPose(self):
  #   kneeFrictionForce = 0
  #   halfpi = 1.57079632679
  #   kneeangle = -2.1834  #halfpi - acos(upper_leg_length / lower_leg_length)
  #
  #   #left front leg
  #   p.resetJointState(self.diff_drive, self.jointNameToId['right_wheel_axle'],
  #                     0.0)
  #   p.resetJointState(self.diff_drive, self.jointNameToId['left_wheel_axle'],
  #                     0.0)
  #
  #   # p.createConstraint(self.diff_drive, self.jointNameToId['knee_front_leftR_link'], self.diff_drive,
  #   #                    self.jointNameToId['knee_front_leftL_link'], p.JOINT_POINT2POINT, [0, 0, 0],
  #   #                    [0, 0.005, 0.2], [0, 0.01, 0.2])
  #   self.setMotorAngleByName('right_wheel_axle', 0.0)
  #   self.setMotorAngleByName('left_wheel_axle', 0.0)
  #
  # def getBasePosition(self):
  #   position, orientation = p.getBasePositionAndOrientation(self.diff_drive)
  #   return position
  #
  # def getBaseOrientation(self):
  #   position, orientation = p.getBasePositionAndOrientation(self.diff_drive)
  #   return orientation
  #
  # def applyAction(self, motorCommands):
  #   for i in range(self.nMotors):
  #     self.setMotorAngleById(self.motorIdList[i], motorCommands[i])
  #
  # def getMotorAngles(self):
  #   motorAngles = []
  #   for i in range(self.nMotors):
  #     jointState = p.getJointState(self.diff_drive, self.motorIdList[i])
  #     motorAngles.append(jointState[0])
  #   motorAngles = np.multiply(motorAngles, self.motorDir)
  #   return motorAngles
  #
  # def getMotorVelocities(self):
  #   motorVelocities = []
  #   for i in range(self.nMotors):
  #     jointState = p.getJointState(self.diff_drive, self.motorIdList[i])
  #     motorVelocities.append(jointState[1])
  #   motorVelocities = np.multiply(motorVelocities, self.motorDir)
  #   return motorVelocities
  #
  # def getMotorTorques(self):
  #   motorTorques = []
  #   for i in range(self.nMotors):
  #     jointState = p.getJointState(self.diff_drive, self.motorIdList[i])
  #     motorTorques.append(jointState[3])
  #   motorTorques = np.multiply(motorTorques, self.motorDir)
  #   return motorTorques
