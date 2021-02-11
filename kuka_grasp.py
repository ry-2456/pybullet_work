# coding: utf-8

# 参考にしたサンプルプログラム
# test-robot-motion.py
# kuka_grasp_block_playback.py
# collisionFIlter.py
# inverse_kinematics.py
# pybullet_robots/baxter_ik_demo.py
# createVisualShapeArray.py -- clickしたobjectのid取得
# segmask_linkindex.py      -- segmaskからobject_id取得

# 参照した情報
# Applying force to grasp an object so that the object does not slip through the gripper #1936
# https://github.com/bulletphysics/bullet3/issues/1936
# https://stackoverflow.com/questions/59128880/getting-world-coordinates-from-opengl-depth-buffer

# link_index == joint_index

import cv2
import pybullet as p
import struct
import time
import pybullet_data
import numpy as np
import math
import sys
import pprint

import matplotlib.pyplot as plt
from sklearn.decomposition import PCA
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
block_ids = []
block_pos = [-0.5, 0, 0]
block_orn = p.getQuaternionFromEuler([0,0,0])
# block_pos = [1., 0, 0.5]
# for i in range(4):
#     # blockId = p.loadURDF("block.urdf", block_pos, block_orn)
#     block_pos[0] -= i*0.1
#     blockId = p.loadURDF("block.urdf", block_pos)
#     block_ids.append(blockId)
block_0_pos = [-0.5, 0.15, 0]
block_1_pos = [-0.5, -0.15, 0]
blockId0 = p.loadURDF("block.urdf", block_0_pos)
blockId1 = p.loadURDF("block.urdf", block_1_pos, p.getQuaternionFromEuler([0,0,np.pi/2]))
block_ids.append(blockId0)
block_ids.append(blockId1)

# p.resetBasePositionAndOrientation(blockId, block_pos, block_orn)
# block_pos[2] = 0.5

# load plane
planeId = p.loadURDF("plane.urdf", useMaximalCoordinates=False)

# load kuka
(kukaId, ) = p.loadSDF("kuka_iiwa/kuka_with_gripper.sdf")
kukaEndEffectorIndex = 6
kukaGripperIndex = 7
leftfing1Id, leftfing2Id = 8, 10
rightfing1Id, rightfing2Id = 11, 13
# p.resetBasePositionAndOrientation(kukaId, [0,0,0], [0,0,0,1])

for b_id in block_ids:
    p.changeDynamics(b_id, -1, lateralFriction=3, frictionAnchor=1)
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

for b_id in block_ids:
    show_coords(b_id, frameLength=0.05)

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

def getRayFromTo(mouseX, mouseY):
    width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera()
    camPos = [
        camTarget[0] - dist * camForward[0], 
        camTarget[1] - dist * camForward[1],
        camTarget[2] - dist * camForward[2]
    ]
    farPlane = 10000
    rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
    invLen = farPlane * 1. / (math.sqrt(rayForward[0] * rayForward[0] + 
                                        rayForward[1] * rayForward[1] + 
                                        rayForward[2] * rayForward[2]))
    rayForward = [invLen * rayForward[0], invLen * rayForward[1], invLen * rayForward[2]]
    rayFrom = camPos
    oneOverWidth = float(1) / float(width)
    oneOverHeight = float(1) / float(height)
    dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
    dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
    rayToCenter = [rayFrom[0] + rayForward[0], rayFrom[1] + rayForward[1], rayFrom[2] + rayForward[2]]
    rayTo = [
        rayFrom[0] + rayForward[0] - 0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] -
        float(mouseY) * dVer[0], rayFrom[1] + rayForward[1] - 0.5 * horizon[1] + 0.5 * vertical[1] +
        float(mouseX) * dHor[1] - float(mouseY) * dVer[1], rayFrom[2] + rayForward[2] -
        0.5 * horizon[2] + 0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]
    ]
    return rayFrom, rayTo

def clicked_object_uid(mouse_x, mouse_y):
    rayFrom, rayTo = getRayFromTo(mouse_x, mouse_y)
    rayInfo = p.rayTest(rayFrom, rayTo)
    hit = rayInfo[0]
    object_uid = hit[0]
    return object_uid

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

def object_mask_from_seg(obj_id, seg):
    """
    segのobj_idのところを255,そうでないところを0
    にしたmaskを返す
    obj_id : body unique id 
    seg : p.getCameraImage(...)の戻り値の4つめの値
    """
    mask = np.zeros(seg.shape, dtype=np.uint8)
    mask[obj_id == seg&((1<<24)-1)] = 255
    return mask

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
    
    return posmap[:,:,:3]

def longitudinal_direction(block_posmap):
    """
    block.urdfの長手方向を求める
    block_posmap : np.array(n x 3)
    return : np.array([float]*3)
    """
    pca = PCA(n_components=3)
    pca.fit(block_posmap)
    v1, v2, v3 = pca.components_[:,[0,1,2]]

    # print("################")
    # print(pca.explained_variance_ratio_)
    # print("################")

    return v1

def get_block_pos(blockId, images):
    """
    block.urdfの中心座標を返す np.array([float]*3)
    """
    color_image_org = np.reshape(images[2], (height, width, 4))
    color_image_ = color_image_org[:,:,[2,1,0]]          # convet RGBA to BGR
    color_image = np.zeros(color_image_.shape, np.uint8) # cv2.circleのエラー回避のため
    copy_img(color_image, color_image_) 
    depth_buffer = np.reshape(images[3], [height, width])
    seg_opengl = np.reshape(images[4], [height, width])

    mask = object_mask_from_seg(blockId, seg_opengl) 
    posmap = get_posmap(near, far, view_matrix, projection_matrix, height, width, depth_buffer)

    # blockの画像中心を求める
    count = np.count_nonzero(mask)
    if count == 0: return None
    row_nonzero_idx, col_nonzero_idx = np.where(mask!=0)
    avev = np.mean(row_nonzero_idx, dtype=np.uint16)
    aveu = np.mean(col_nonzero_idx, dtype=np.uint16)

    cv2.circle(color_image, (aveu,avev), 3, (255, 255, 0), -1)
    cv2.imshow("color image", color_image)
    cv2.imshow("mask", mask)
    cv2.waitKey(10)

    block_center = posmap[avev, aveu]
    return block_center

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
    # is_grasping = True
    # grasp()
    is_grasping = False
    release()

    eef_pos, eef_orn = p.getLinkState(kukaId, kukaEndEffectorIndex)[4:6]
    eef_orn = p.getEulerFromQuaternion(eef_orn)
    eef_pos = np.array(eef_pos)
    eef_orn = np.array(eef_orn)

    done_init = False
    while True:

        if not done_init:
            eef_pos[2] = 0.5
            move_eff(eef_pos, [0,-np.pi,0])
            done_init = True
        
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

        images = p.getCameraImage(width,
                                  height,
                                  view_matrix,
                                  projection_matrix,
                                  # shadow=True,
                                  renderer=p.ER_BULLET_HARDWARE_OPENGL)

        # クリックされたobject_uidを取得
        object_uid = -1
        mouse_events = p.getMouseEvents()
        for e in mouse_events:
            if ((e[0] == 2) and (e[3] == 0) and (e[4] & p.KEY_WAS_TRIGGERED)): # 左クリック判定
                mouse_x = e[1]
                mouse_y = e[2]
                object_uid = clicked_object_uid(mouse_x, mouse_y) # -1 for no object was clicked 
        
        # print(object_uid)
        if object_uid < 0: continue # no object was clicked 
        if object_uid not in block_ids: continue # a block was not clicked 
        
        b_pos = get_block_pos(object_uid, images)
        if b_pos is None: continue

        if object_uid == blockId0: print(block_0_pos)
        if object_uid == blockId1: print(block_1_pos)
        print(b_pos)

        # ブロックの長手方向の計算
        depth_buffer = images[3].reshape(height, width)
        seg = images[4].reshape(height, width)
        block_mask = object_mask_from_seg(object_uid, seg)
        posmap = get_posmap(near, far, 
                            view_matrix, projection_matrix, 
                            height, width, depth_buffer)
        block_posmap = posmap[block_mask!=0]
        v = longitudinal_direction(block_posmap)

        # world -> eefの同時変換行列を計算
        eef_base_pos, eef_base_orn = p.getLinkState(kukaId, kukaEndEffectorIndex)[4:6]
        world_to_eef_base = np.eye(4)
        world_to_eef_base[0:3,0:3] = np.array(p.getMatrixFromQuaternion(eef_base_orn)).reshape(3,3)
        world_to_eef_base[:3,3] = eef_base_pos

        # yawの回転角の計算, eefのy軸が方向にブロックの長手方向が向いていればいい
        v_in_eef = np.linalg.inv(world_to_eef_base).dot(np.hstack([v,1]))[:3] - np.linalg.inv(world_to_eef_base).dot(np.array([0,0,0,1]))[:3]
        v_in_eef = v_in_eef / np.linalg.norm(v_in_eef)
        e_y = np.array([0,1.,0])
        yaw_rot_diff = np.arccos(np.dot(v_in_eef, e_y))
        # print(yaw_rot_diff)
        # print(np.rad2deg(yaw_rot_diff))
        yaw_now = p.getEulerFromQuaternion(eef_base_orn)[2]
        eef_orn = [0, -np.pi, yaw_now] # eefが下向き
        if v_in_eef[0] < 0: eef_orn[2] -= yaw_rot_diff
        else:               eef_orn[2] += yaw_rot_diff
        # TODO : eef_orn[2]を範囲に収める処理を追加

        release()
        time.sleep(0.1)
        move_eff([-0.5,0,0.5], eef_orn)
        time.sleep(2)
        b_pos[2] = -0.005+0.1
        move_eff(b_pos, eef_orn)
        time.sleep(2)
        b_pos[2] = -0.005
        move_eff(b_pos, eef_orn)
        time.sleep(2)
        grasp()
        time.sleep(0.1)
    
        # # blockの長手方向を描画
        p.addUserDebugLine(b_pos+v/10, b_pos-v/10, [1, 0, 0])
