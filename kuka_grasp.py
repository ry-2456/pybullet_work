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
# link_index == joint_index

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
kukaEndEffectorIndex = 6
kukaGripperIndex = 7
leftfing1Id, leftfing2Id = 8, 10
rightfing1Id, rightfing2Id = 11, 13
# p.resetBasePositionAndOrientation(kukaId, [0,0,0], [0,0,0,1])

p.changeDynamics(blockId, -1, lateralFriction=3, frictionAnchor=1)
p.changeDynamics(kukaId, leftfing2Id, lateralFriction=3, frictionAnchor=1)
p.changeDynamics(kukaId, rightfing2Id, lateralFriction=3, frictionAnchor=1)
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
useNullSpace = 1

useOrientation = 1
#If we set useSimulation=0, it sets the arm pose to be the IK result directly without using dynamic control.
#This can be used to test the IK result accuracy.
useSimulation = 1
useRealTimeSimulation = 1
ikSolver = 0
p.setRealTimeSimulation(useRealTimeSimulation)

def grasp():
    leftfing1Id, leftfing2Id = 8, 10
    rightfing1Id, rightfing2Id = 11, 13

    grasp_lf1_rot = 0
    grasp_rf1_rot = 0

    p.setJointMotorControlArray(bodyIndex=kukaId,
                                jointIndices=[leftfing1Id, rightfing1Id],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[grasp_lf1_rot, grasp_rf1_rot],
                                targetVelocities=[0, 0],
                                forces=[500, 500],
                                positionGains=[0.03, 0.03],
                                velocityGains=[1, 1])

def release():
    leftfing1Id, leftfing2Id = 8, 10
    rightfing1Id, rightfing2Id = 11, 13

    release_lf1_rot = -0.165
    release_rf1_rot = 0.165

    p.setJointMotorControlArray(bodyIndex=kukaId,
                                jointIndices=[leftfing1Id, rightfing1Id],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[release_lf1_rot, release_rf1_rot],
                                targetVelocities=[0, 0],
                                forces=[500, 500],
                                positionGains=[0.03, 0.03],
                                velocityGains=[1, 1])

is_grasping = True

def test_ik():

    # kukaEndEffectorIndexからleftfing2Id(左の手先)までのz軸方の距離を求める
    endEffector_pos = p.getLinkState(kukaId, kukaEndEffectorIndex)
    leftTip_pos = p.getLinkState(kukaId, leftfing2Id)
    x_diff = endEffector_pos[4][0] - leftTip_pos[4][0]
    y_diff = endEffector_pos[4][1] - leftTip_pos[4][1]
    z_diff = endEffector_pos[4][2] - leftTip_pos[4][2]

    orn = p.getQuaternionFromEuler([0, -math.pi, math.pi/2.])
    pos = block_pos[:2] + [0] # zは0
    pos[2] += z_diff+0.080 # 手先

    targetPosXId = p.addUserDebugParameter("targetPosX", -1, 1, pos[0])
    targetPosYId = p.addUserDebugParameter("targetPosY", -1, 1, pos[1])
    targetPosZId = p.addUserDebugParameter("targetPosZ", -1, 1, pos[2])

    # for gripper 
    eef_init = 0
    eef_rot_id = p.addUserDebugParameter("eef", -3.14, 3.14, eef_init)

    cnt = 1
    while True:

        targetPosX = p.readUserDebugParameter(targetPosXId)
        targetPosY = p.readUserDebugParameter(targetPosYId)
        targetPosZ = p.readUserDebugParameter(targetPosZId)
        # targetAngleGrasp = p.readUserDebugParameter(targetAngleGraspId)
        # targetAngleTip = p.readUserDebugParameter(targetAngleTipId)

        eef_rot = p.readUserDebugParameter(eef_rot_id)

        pos = [targetPosX, targetPosY, targetPosZ]
        jointPoses = p.calculateInverseKinematics(kukaId,
                                                  kukaEndEffectorIndex,
                                                  pos,
                                                  orn,
                                                  lowerLimits=ll,
                                                  upperLimits=ul,
                                                  jointRanges=jr,
                                                  restPoses=rp)

        numJoints = p.getNumJoints(kukaId)

        keys = p.getKeyboardEvents()
        ENTER = 65309
        global is_grasping
        if ENTER in keys and keys[ENTER]&p.KEY_WAS_RELEASED:
            if is_grasping:
                release()
                is_grasping = False
            else:
                grasp()
                is_grasping = True


if __name__ == "__main__":
    test_ik()
