# coding: utf-8

# 参考にしたサンプルプログラム
# test-robot-motion.py
# kuka_grasp_block_playback.py
# collisionFIlter.py
# inverse_kinematics.py
# pybullet_robots/baxter_ik_demo.py

# 参照した情報
# Applying force to grasp an object so that the object does not slip through the gripper #1936
# https://github.com/bulletphysics/bullet3/issues/1936

from datetime import datetime
import pybullet as p
import struct
import pybullet_data
import numpy as np
import math
import sys
import pprint

from show_coords import show_coords

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10.)

# load block
block_pos = [-0.5, 0, 0]
block_orn = p.getQuaternionFromEuler([0,0,0])
# block_pos = [1., 0, 0.5]
blockId = p.loadURDF("block.urdf", block_pos, block_orn)
p.resetBasePositionAndOrientation(blockId, block_pos, block_orn)
block_pos[2] = 0.5

# load plane
planeId = p.loadURDF("plane.urdf", useMaximalCoordinates=False)

# load kuka
(kukaId, ) = p.loadSDF("kuka_iiwa/kuka_with_gripper.sdf")
# numJoints = p.getNumJoints(kukaId)
numJoints = 7
kukaEndEffectorIndex = 6
kukaGripperIndex = 7
leftfing1Id, leftfing2Id = 8, 10
rightfing1Id, rightfing2Id = 11, 13
# p.resetBasePositionAndOrientation(kukaId, [0,0,0], [0,0,0,1])

# for i in range(numJoints):
#     p.resetJointState(kukaId, i, np.pi/4)
def print_friction_coefficient(objectId, linkIndex):
    # print lateral, rolling and spinning coefficient
    dynamics_info = p.getDynamicsInfo(objectId, linkIndex)
    lateral_friction_coeff = dynamics_info[1]
    rolling_friction_coeff = dynamics_info[6]
    spinning_friction_coeff = dynamics_info[7]
    print("lateral_friction_coeff:", lateral_friction_coeff)
    print("rolling_friction_coeff:", rolling_friction_coeff)
    print("spinning_friction_coeff:", spinning_friction_coeff)

# lateralFrictionを大きくし、frictionAnchorを有効にするとグリップからものが滑りにくくなる
# frictionAnchorが何なのかはわからない
p.changeDynamics(blockId, -1, lateralFriction=3, frictionAnchor=1)
p.changeDynamics(kukaId, leftfing2Id, lateralFriction=3, frictionAnchor=1)
p.changeDynamics(kukaId, rightfing2Id, lateralFriction=3, frictionAnchor=1)

# p.setCollisionFilterPair(
#             bodyUniqueIdA=planeId, 
#             bodyUniqueIdB=blockId, 
#             linkIndexA=-1, 
#             linkIndexB=-1, 
#             enableCollision=1) # set 1 to enable, set 0 to disable

objectNum = p.getNumBodies()

for i in range(p.getNumJoints(kukaId)):
    show_coords(kukaId, parentLinkIndex=i, frameLength=0.08)
    p.addUserDebugText("link:{}".format(i), 
                        textPosition=[0.01,0,0],
                        textColorRGB=[1,0,0],
                        textSize=1,
                        parentObjectUniqueId=kukaId,
                        parentLinkIndex=i)
    # pprint.pprint(p.getJointInfo(kukaId, i)[3])
    print(p.getJointInfo(kukaId, i)[3])
show_coords(blockId, frameLength=0.05)

# def Step(stepIndex):
#   for objectId in range(objectNum):
#     record = log[stepIndex * objectNum + objectId]
#     Id = record[2]
#     pos = [record[3], record[4], record[5]]
#     orn = [record[6], record[7], record[8], record[9]]
#     p.resetBasePositionAndOrientation(Id, pos, orn)
#     numJoints = p.getNumJoints(Id)
#     for i in range(numJoints):
#       jointInfo = p.getJointInfo(Id, i)
#       qIndex = jointInfo[3]
#       if qIndex > -1:
#         p.resetJointState(Id, i, record[qIndex - 7 + 17])


# stepIndexId = p.addUserDebugParameter("stepIndex", 0, recordNum / objectNum - 1, 0)

# while True:
#   stepIndex = int(p.readUserDebugParameter(stepIndexId))
#   Step(stepIndex)
#   p.stepSimulation()
#   Step(stepIndex)
#lower limits for null space
ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
#upper limits for null space
ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
#joint ranges for null space
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
#restposes for null space
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
#joint damping coefficents
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

for i in range(len(rp)):
    p.resetJointState(kukaId, i, rp[i])

t = 0.
# prevPose = [0, 0, 0]
# prevPose1 = [0, 0, 0]
# hasPrevPose = 0
useNullSpace = 1

useOrientation = 1
#If we set useSimulation=0, it sets the arm pose to be the IK result directly without using dynamic control.
#This can be used to test the IK result accuracy.
useSimulation = 1
useRealTimeSimulation = 1
ikSolver = 0
p.setRealTimeSimulation(useRealTimeSimulation)

# while p.isConnected(physicsClient):
#     # p.stepSimulation()
#     pass
def test_ik():

    # kukaEndEffectorIndexからleftfing2Id(左の手先)までのz軸方の距離を求める
    endEffector_pos = p.getLinkState(kukaId, kukaEndEffectorIndex)
    leftTip_pos = p.getLinkState(kukaId, leftfing2Id)
    x_diff = endEffector_pos[4][0] - leftTip_pos[4][0]
    y_diff = endEffector_pos[4][1] - leftTip_pos[4][1]
    z_diff = endEffector_pos[4][2] - leftTip_pos[4][2]
    # print("x_diff :", x_diff)
    # print("y_diff :", y_diff)
    # print("z_diff :", z_diff)
    # print("#####################")
    # print(endEffector_pos)
    # print(endEffector_pos[4][0])
    # print(endEffector_pos[2])
    # print(leftTip_pos[4][0])
    # print(leftTip_pos[2])
    # print("#####################")

    # orn = p.getQuaternionFromEuler([0, -math.pi, 0])
    orn = p.getQuaternionFromEuler([0, -math.pi, math.pi/2.])
    pos = block_pos[:2] + [0] # zは0
    pos[2] += z_diff+0.080 # 手先

    targetPosXId = p.addUserDebugParameter("targetPosX", -1, 1, pos[0])
    targetPosYId = p.addUserDebugParameter("targetPosY", -1, 1, pos[1])
    targetPosZId = p.addUserDebugParameter("targetPosZ", -1, 1, pos[2])
    # targetAngleGraspId = p.addUserDebugParameter("targetAngleGrasp", -1, 1, pos[2])
    # targetAngleTipId = p.addUserDebugParameter("targetAngleTip", -1, 1, pos[2])

    cnt = 1
    while True:
        targetPosX = p.readUserDebugParameter(targetPosXId)
        targetPosY = p.readUserDebugParameter(targetPosYId)
        targetPosZ = p.readUserDebugParameter(targetPosZId)
        # targetAngleGrasp = p.readUserDebugParameter(targetAngleGraspId)
        # targetAngleTip = p.readUserDebugParameter(targetAngleTipId)

        pos = [targetPosX, targetPosY, targetPosZ]
        # print("targetPosX: ", targetPosX)
        # print("targetPosY: ", targetPosY)
        # print("targetPosZ: ", targetPosZ)
        jointPoses = p.calculateInverseKinematics(kukaId,
                                                  kukaEndEffectorIndex,
                                                  pos,
                                                  orn,
                                                  lowerLimits=ll,
                                                  upperLimits=ul,
                                                  jointRanges=jr,
                                                  restPoses=rp)

        numJoints = p.getNumJoints(kukaId)
        # print("numJoints: ", numJoints)
        skiped_num = 0
        for i in range(numJoints):
            jointInfo = p.getJointInfo(kukaId, i)
            qIndex = jointInfo[3]
            if qIndex == -1:
                skiped_num += 1
                continue
            # if qIndex > -1:
            jointPoses = list(jointPoses)
            # 手首
            if i == leftfing1Id:
                # jointPoses[i-skiped_num] = -math.pi/(32+cnt)
                jointPoses[i-skiped_num] = math.pi/64 - math.pi/cnt
                # jointPoses[i-skiped_num] = 0
            if i == rightfing1Id:
                #jointPoses[i-skiped_num] = math.pi/(32+cnt)
                jointPoses[i-skiped_num] = -math.pi/64 + math.pi/cnt
                # jointPoses[i-skiped_num] = 0
            # 手先
            if i == leftfing2Id:
                jointPoses[i-skiped_num] = -math.pi/16
                # jointPoses[i-skiped_num] = 0
            if i == rightfing2Id:
                jointPoses[i-skiped_num] = math.pi/16
                # jointPoses[i-skiped_num] = 0

            # print(jointPoses)
                
            p.setJointMotorControl2(bodyIndex=kukaId,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[i-skiped_num],
                                    targetVelocity=0,
                                    force=500,
                                    positionGain=0.03,
                                    velocityGain=1)

        # for i in range(numJoints):
        #   p.setJointMotorControl2(bodyIndex=kukaId,
        #                           jointIndex=i,
        #                           controlMode=p.POSITION_CONTROL,
        #                           targetPosition=jointPoses[i],
        #                           targetVelocity=0,
        #                           force=500,
        #                           positionGain=0.03,
        #                           velocityGain=1)
        # print(jointPoses)
        cnt += 1
test_ik()

sys.exit(0)

i = 0
while 1:
  i+=1
  if (useRealTimeSimulation):
    dt = datetime.now()
    t = (dt.second / 60.) * 2. * math.pi
  else:
    t = t + 0.01

  if (useSimulation and useRealTimeSimulation == 0):
    p.stepSimulation()

  for i in range(1):
    pos = [-0.4, 0.2 * math.cos(t), 0. + 0.2 * math.sin(t)]
    #end effector points down, not up (in case useOrientation==1)
    orn = p.getQuaternionFromEuler([0, -math.pi, 0])

    if (useNullSpace == 1):
      if (useOrientation == 1):
        jointPoses = p.calculateInverseKinematics(
                            kukaId, 
                            kukaEndEffectorIndex, 
                            pos, orn, ll, ul, jr, rp)
      else:
        jointPoses = p.calculateInverseKinematics(kukaId,
                                                  kukaEndEffectorIndex,
                                                  pos,
                                                  lowerLimits=ll,
                                                  upperLimits=ul,
                                                  jointRanges=jr,
                                                  restPoses=rp)
    else:
      if (useOrientation == 1):
        jointPoses = p.calculateInverseKinematics(kukaId,
                                                  kukaEndEffectorIndex,
                                                  pos,
                                                  orn,
                                                  jointDamping=jd,
                                                  solver=ikSolver,
                                                  maxNumIterations=100,
                                                  residualThreshold=.01)
      else:
        jointPoses = p.calculateInverseKinematics(kukaId,
                                                  kukaEndEffectorIndex,
                                                  pos,
                                                  solver=ikSolver)

    if (useSimulation):
      for i in range(numJoints):
        p.setJointMotorControl2(bodyIndex=kukaId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointPoses[i],
                                targetVelocity=0,
                                force=500,
                                positionGain=0.03,
                                velocityGain=1)
    else:
      #reset the joint state (ignoring all dynamics, not recommended to use during simulation)
      for i in range(numJoints):
        p.resetJointState(kukaId, i, jointPoses[i])

# p.disconnect()

