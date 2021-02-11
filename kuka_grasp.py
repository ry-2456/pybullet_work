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

import cv2
import pybullet as p
import struct
import pybullet_data
import numpy as np
import math
import sys
import pprint

from datetime import datetime
from show_coords import show_coords

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10.)

# for camera
width, height, fov = 240, 240, 60
aspect = width / height
near, far = 0.02, 4
view_matrix = p.computeViewMatrix([-0.6, 0, 0.6], [-0.5, 0, 0], [1, 0, 0])
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

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

def copy_img(dst, src):
    """
    p.getCameraImageで得た画像をcv2.circleに渡すとerrorが起きる

    dst = np.array(src.shpae, src.dtype)
    copy_img(dst, src)
    cv2.circle(dst, ...)

    としてdstをcv2.circleに渡すとerrorが起きなくなる
    errorの原因は不明
    """
    img_h, img_w = src.shape[:2]
    for i in range(img_h): 
        for j in range(img_w):
            dst[i,j] = src[i,j]

def get_posmap(near, far, view_matrix, projection_matrix, height, width, depth_buffer):
    posmap = np.empty([height, width, 4])
    projectionMatrix = np.asarray(projection_matrix).reshape([4,4],order='F')
    viewMatrix = np.asarray(view_matrix).reshape([4,4],order='F')
    tran_pix_world = np.linalg.inv(np.matmul(projectionMatrix, viewMatrix))
    for h in range(height):
        for w in range(width):
            x = (2.*w - width)/width
            y = -(2.*h - height)/height  # be careful！ deepth and its corresponding position
            z = 2.*depth_buffer[h,w] - 1
            pixPos = np.asarray([x, y, z, 1])
            position = np.matmul(tran_pix_world, pixPos)
            posmap[h,w,:] = position / position[3]
    
    return posmap

def get_block_pos(blockId, images):
    """
    block.urdfの中心座標を返す np.array([float]*3)
    """
    color_image_org = np.reshape(images[2], (height, width, 4))
    color_image_ = color_image_org[:,:,[2,1,0]] # convet RGBA to BGR
    # color_image = color_image.astype(np.uint8) # convert int32 to uint8.
    color_image = np.zeros(color_image_.shape, np.uint8) # color_image_にcv2.circleで書き込むを原因不明の
    copy_img(color_image, color_image_) # エラーが起きるこのように左のようにすればエラー回避できる
    depth_buffer = np.reshape(images[3], [height, width])
    depth_image = near / (far - (far - near) * depth_buffer)
    seg_opengl = np.reshape(images[4], [height, width])
    # flags = (images[4] == blockId)
    flags = (seg_opengl == blockId)
    
    aveu, avev, count = 0, 0, 0
    for i in range(height):
        for j in range(width):
            # if flags[i][j] == True:
                object_uid = seg_opengl[i,j] & ((1 << 24) - 1)
                if object_uid == blockId:
                    aveu += j
                    avev += i
                    count += 1

    if count == 0:
        return None

    posmap = get_posmap(near, far, view_matrix, projection_matrix, height, width, depth_buffer)
    print(posmap[int(avev/count), int(aveu/count)])
    print(block_pos)
  
    vm = np.array(view_matrix).reshape(4,4)
    aveu = aveu/count - width/2
    avev = avev/count - height/2
    theta_x = (np.deg2rad(fov)*aveu)/(2*height)
    theta_y = (np.deg2rad(fov)*avev)/(2*height)
    z = -depth_buffer[int(avev)][int(aveu)]
    x = -z*np.tan(theta_x)
    y = -z*np.tan(theta_y)
    x_camera = np.array((x, y, z, 1))
    x_world = np.dot(np.linalg.inv(vm.T), x_camera)
    
    center = (int(aveu + width/2), int(avev + height/2))
    cv2.circle(color_image, center, 3, (255, 255, 0), -1)
    # cv2.circle(color_image, center, 3, (255, 255, 0), -1)
    img = np.zeros(color_image.shape, np.uint8)
    cv2.circle(img, (int(aveu + width/2), int(avev + height/2)), 3, (255, 255, 0), -1)
    cv2.imshow("depth image", depth_image)
    cv2.imshow("color image", color_image)
    cv2.waitKey(10)

    return x_world[:3]

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
    target_pos[2] -= z_offset-0.063 # TODO: もう少し調整が必要0.056
                                    #       角度がついた時も考える

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
        # グリッパーの関節角は変えないt
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
    grasp()

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

        mouse_events = p.getMouseEvents()
        if mouse_events:
            print(mouse_events)

        images = p.getCameraImage(width,
                                  height,
                                  view_matrix,
                                  projection_matrix,
                                  # shadow=True,
                                  renderer=p.ER_BULLET_HARDWARE_OPENGL)
        
        b_pos = get_block_pos(blockId, images)
        print(b_pos.shape)
        # print("##########")
        # print(b_pos)
        # print(block_pos)
        # print("##########")
