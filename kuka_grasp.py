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
    grasp_lf2_rot = 0
    grasp_rf2_rot = 0

    joint_indices = [leftfing1Id, rightfing1Id, leftfing2Id, rightfing2Id]
    target_positions = [grasp_lf1_rot, grasp_rf1_rot, 
                        grasp_lf2_rot, grasp_rf2_rot]

    p.setJointMotorControlArray(bodyIndex=kukaId,
                                jointIndices=joint_indices,
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=target_positions,
                                targetVelocities=[0]*4,
                                forces=[500]*4,
                                positionGains=[0.03]*4,
                                velocityGains=[1]*4)

def release():
    leftfing1Id, leftfing2Id = 8, 10
    rightfing1Id, rightfing2Id = 11, 13

    release_lf1_rot = -0.165
    release_rf1_rot = 0.165 
    release_lf2_rot = 0
    release_rf2_rot = 0

    joint_indices = [leftfing1Id, rightfing1Id, leftfing2Id, rightfing2Id]
    target_positions = [release_lf1_rot, release_rf1_rot, 
                        release_lf2_rot, release_rf2_rot]

    p.setJointMotorControlArray(bodyIndex=kukaId,
                                jointIndices=joint_indices,
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=target_positions,
                                targetVelocities=[0]*4,
                                forces=[500]*4,
                                positionGains=[0.03]*4,
                                velocityGains=[1]*4)

def move_eff(target_pos, target_orn):
    # kukaEndEffectorIndexからleftfing2Id(左の手先)までのz軸方の距離を求める
    # TODO:
    # 同時変換行列で計算しましょう
    # エンドエフェクタのベースと手先のフレームの同時変換行列から計算できる
    # 手先をある姿勢,位置にするにはエンドエフェクタのベースをどこに持っていけばいいか
    # これを計算する
    # getMatrixFromQuaternion
    eef_base_pos, eef_base_orn = p.getLinkState(kukaId, kukaEndEffectorIndex)[4:6]
    left_tip_pos, left_tip_orn = p.getLinkState(kukaId, leftfing2Id)[4:6]
    right_tip_pos, right_tip_orn = p.getLinkState(kukaId, rightfing2Id)[4:6]

    x_offset = (left_tip_pos[0] + right_tip_pos[0])/2.0 - eef_base_pos[0]
    y_offset = (left_tip_pos[1] + right_tip_pos[1])/2.0 - eef_base_pos[1]
    z_offset = (left_tip_pos[2] + right_tip_pos[2])/2.0 - eef_base_pos[2]
    target_pos[0] -= x_offset
    target_pos[1] -= y_offset
    target_pos[2] -= z_offset-0.03 # TODO: もう少し調整が必要

    # eef_base_to_tip = np.eye(4)
    # eef_base_to_tip[:3,3] = [x_offset, y_offset, z_offset]
    # world_to_eef_base = np.eye(4)
    # world_to_eef_base[0:3,0:3] = np.array(p.getMatrixFromQuaternion(eef_base_orn)).reshape(3,3)
    # world_to_eef_base[:3,3] = eef_base_pos

    # x_diff = endEffector_pos[4][0] - leftTip_pos[4][0]
    # y_diff = endEffector_pos[4][1] - leftTip_pos[4][1]
    # z_diff = endEffector_pos[4][2] - leftTip_pos[4][2]
    # orn = p.getQuaternionFromEuler([0, -math.pi, math.pi/2.])
    # pos = block_pos[:2] + [0] # zは0
    # pos[2] += z_diff+0.080 # 手先
    
    # yaw->pitch->rollの順番で回転
    if len(target_orn) == 3:
        target_orn= p.getQuaternionFromEuler(target_orn)

    jointPoses = p.calculateInverseKinematics(kukaId,
                                              kukaEndEffectorIndex,
                                              target_pos,
                                              target_orn,
                                              lowerLimits=ll,
                                              upperLimits=ul,
                                              jointRanges=jr,
                                              restPoses=rp)

    # move eef
    numJoints = p.getNumJoints(kukaId)
    skiped_num = 0
    for i in range(numJoints):
        jointInfo = p.getJointInfo(kukaId, i)
        qIndex = jointInfo[3]
        if qIndex == -1:
            skiped_num += 1
            continue
        jointPoses = list(jointPoses)
        # TODO: setJointMotorControlArrayに変更する
        # グリッパーの関節角は変えない
        if i in [leftfing1Id, leftfing2Id, rightfing1Id, rightfing2Id]:
            continue
        p.setJointMotorControl2(bodyIndex=kukaId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointPoses[i-skiped_num],
                                targetVelocity=0,
                                force=500,
                                positionGain=0.03,
                                velocityGain=1)


if __name__ == "__main__":
    is_grasping = True

    eef_pos, eef_orn = p.getLinkState(kukaId, kukaEndEffectorIndex)[4:6]
    eef_orn = p.getEulerFromQuaternion(eef_orn)

    eef_x_id = p.addUserDebugParameter("eef_x", -1, 2, eef_pos[0])
    eef_y_id = p.addUserDebugParameter("eef_y", -1, 2, eef_pos[1])
    eef_z_id = p.addUserDebugParameter("eef_z", -1, 2, eef_pos[2])
    eef_roll_id  = p.addUserDebugParameter("eef_roll", -math.pi, math.pi, eef_orn[0])
    eef_pitch_id = p.addUserDebugParameter("eef_pitch", -math.pi, math.pi, eef_orn[1])
    eef_yaw_id   = p.addUserDebugParameter("eef_yaw", -math.pi, math.pi, eef_orn[2])

    # for gripper 
    eef_init = 0
    eef_rot_id = p.addUserDebugParameter("eef", -3.14, 3.14, eef_init)

    cnt = 1
    while True:
        eef_x = p.readUserDebugParameter(eef_x_id)
        eef_y = p.readUserDebugParameter(eef_y_id)
        eef_z = p.readUserDebugParameter(eef_z_id)
        eef_roll  = p.readUserDebugParameter(eef_roll_id)
        eef_pitch = p.readUserDebugParameter(eef_pitch_id)
        eef_yaw   = p.readUserDebugParameter(eef_yaw_id)
        # targetAngleGrasp = p.readUserDebugParameter(targetAngleGraspId)
        # targetAngleTip = p.readUserDebugParameter(targetAngleTipId)
        pos = [eef_x, eef_y, eef_z]
        orn = [eef_roll, eef_pitch, eef_yaw]
        move_eff(pos, orn)
        
        eef_rot = p.readUserDebugParameter(eef_rot_id)
        numJoints = p.getNumJoints(kukaId)
        keys = p.getKeyboardEvents()
        ENTER = 65309
        if ENTER in keys and keys[ENTER]&p.KEY_WAS_RELEASED:
            if is_grasping:
                release()
                is_grasping = False
            else:
                grasp()
                is_grasping = True
