import pybullet as p
import cv2
import time
import pybullet_data
import numpy as np
import math
import sys

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

def get_object_uid(mouse_x, mouse_y):
    rayFrom, rayTo = getRayFromTo(mouse_x, mouse_y)
    rayInfo = p.rayTest(rayFrom, rayTo)
    hit = rayInfo[0]
    object_uid = hit[0]
    return object_uid

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
