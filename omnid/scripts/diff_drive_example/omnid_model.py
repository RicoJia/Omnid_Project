import pybullet as p
import numpy as np

class Omnid_Model:

  def __init__(self, urdfRootPath=''):
    self.urdfRootPath = urdfRootPath
    self.reset()
      
  def buildJointNameToIdDict(self):
    nJoints = p.getNumJoints(self.diff_drive)
    self.jointNameToId = {}
    for i in range(nJoints):
      jointInfo = p.getJointInfo(self.diff_drive, i)
      self.jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    self.resetPose()
    for i in range(100):
      p.stepSimulation()

  def buildMotorIdList(self):
    self.motorIdList.append(self.jointNameToId["right_wheel_axle"])
    self.motorIdList.append(self.jointNameToId["left_wheel_axle"])

  def reset(self):
    self.diff_drive = p.loadURDF("%s/diff_drive.urdf" % self.urdfRootPath, 0, 0, 0)

    self.kp = 1
    self.kd = 0.1
    self.maxForce = 35
    self.nMotors = 2
    self.motorIdList = []
    self.motorDir = [-1, -1]
    self.buildJointNameToIdDict()
    self.buildMotorIdList()

  def setMotorAngleById(self, motorId, desiredAngle):
    p.setJointMotorControl2(bodyIndex=self.diff_drive,
                            jointIndex=motorId,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=desiredAngle,
                            positionGain=self.kp,
                            velocityGain=self.kd,
                            force=self.maxForce)

  def setMotorAngleByName(self, motorName, desiredAngle):
    self.setMotorAngleById(self.jointNameToId[motorName], desiredAngle)

  def resetPose(self):
    kneeFrictionForce = 0
    halfpi = 1.57079632679
    kneeangle = -2.1834  #halfpi - acos(upper_leg_length / lower_leg_length)

    #left front leg
    p.resetJointState(self.diff_drive, self.jointNameToId['right_wheel_axle'],
                      0.0)
    p.resetJointState(self.diff_drive, self.jointNameToId['left_wheel_axle'],
                      0.0)

    # p.createConstraint(self.diff_drive, self.jointNameToId['knee_front_leftR_link'], self.diff_drive,
    #                    self.jointNameToId['knee_front_leftL_link'], p.JOINT_POINT2POINT, [0, 0, 0],
    #                    [0, 0.005, 0.2], [0, 0.01, 0.2])
    self.setMotorAngleByName('right_wheel_axle', 0.0)
    self.setMotorAngleByName('left_wheel_axle', 0.0)

  def getBasePosition(self):
    position, orientation = p.getBasePositionAndOrientation(self.diff_drive)
    return position

  def getBaseOrientation(self):
    position, orientation = p.getBasePositionAndOrientation(self.diff_drive)
    return orientation

  def applyAction(self, motorCommands):
    for i in range(self.nMotors):
      self.setMotorAngleById(self.motorIdList[i], motorCommands[i])

  def getMotorAngles(self):
    motorAngles = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.diff_drive, self.motorIdList[i])
      motorAngles.append(jointState[0])
    motorAngles = np.multiply(motorAngles, self.motorDir)
    return motorAngles

  def getMotorVelocities(self):
    motorVelocities = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.diff_drive, self.motorIdList[i])
      motorVelocities.append(jointState[1])
    motorVelocities = np.multiply(motorVelocities, self.motorDir)
    return motorVelocities

  def getMotorTorques(self):
    motorTorques = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.diff_drive, self.motorIdList[i])
      motorTorques.append(jointState[3])
    motorTorques = np.multiply(motorTorques, self.motorDir)
    return motorTorques
