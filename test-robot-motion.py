import os, time, math
import numpy as np
import cv2
from datetime import datetime
import pybullet as p
import pybullet_data

####################### Initial settings ########################
# for pysical simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setAdditionalSearchPath(os.environ["HOME"] + "/bullet3/data")
# p.setAdditionalSearchPath(os.environ["PYBULLET_DATA_PATH"])
p.loadURDF("plane.urdf", [0, 0, -0.3], useFixedBase=True)

# for camera
width, height, fov = 240, 240, 60
aspect = width / height
near, far = 0.02, 4
view_matrix = p.computeViewMatrix([-0.6, 0, 0.6], [-0.5, 0, 0], [1, 0, 0])
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

# for simulation environment
(kukaId, ) = p.loadSDF("kuka_iiwa/kuka_with_gripper.sdf")
trayId = p.loadURDF("tray/tray.urdf", [-0.5, 0, -0.302],
                    flags=p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT)

blockIds = []
block_pos = [-0.55, 0.11, -0.2]
for i in range(1):
    # blockIds.append(p.loadURDF("block.urdf", [-0.5, 0, 1 + 0.1*float(i)],
    #                            flags=p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT))
    blockIds.append(p.loadURDF("block.urdf", block_pos,
                               flags=p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT,
                               useFixedBase=True))
# p.setGravity(0, 0, -10)

enableCollision = 1
p.setCollisionFilterPair(trayId, blockIds[0], -1, -1, enableCollision)

# for Kuka arm
p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])
kukaEndEffectorIndex = 6
numJoints = 7 
leftfing1Id, leftfing2Id = 8, 10
rightfing1Id, rightfing2Id = 11, 13
ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]         #lower limits for null space
ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]               #upper limits for null space
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]                           #joint ranges for null space
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0] #restposes for null space
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]                   #joint damping coefficents
for i in range(numJoints):
    p.resetJointState(kukaId, i, rp[i])

t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
useNullSpace = 0
################################################################

count = 0
useSimulation = 1
useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)
#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
trailDuration = 15

def copy_img(dst, src):
    img_h, img_w = src.shape[:2]
    for i in range(img_h):                              # エラーが起きるこのように左のようにすればエラー回避できる
        for j in range(img_w):
            dst[i,j] = src[i,j]

def getBlockPos2(blockId, images):
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

    stepX = 1
    stepY = 1
    # pointCloud = np.empty([np.int(img_height/stepY), np.int(img_width/stepX), 4])
    # pointCloud = np.empty([np.int(height/stepY), np.int(width/stepX), 4])
    pointCloud = np.empty([int(height/stepY), int(width/stepX), 4])
    projectionMatrix = np.asarray(projection_matrix).reshape([4,4],order='F')
    viewMatrix = np.asarray(view_matrix).reshape([4,4],order='F')
    tran_pix_world = np.linalg.inv(np.matmul(projectionMatrix, viewMatrix))
    # for h in range(0, img_height, stepY):
    #     for w in range(0, img_width, stepX):
    for h in range(0, height, stepY):
        for w in range(0, width, stepX):
            x = (2*w - width)/width
            y = -(2*h - height)/height  # be careful！ deepth and its corresponding position
            # z = 2*depth_np_arr[h,w] - 1
            z = 2*depth_buffer[h,w] - 1
            pixPos = np.asarray([x, y, z, 1])
            position = np.matmul(tran_pix_world, pixPos)

            # pointCloud[np.int(h/stepY),np.int(w/stepX),:] = position / position[3]
            pointCloud[int(h/stepY),int(w/stepX),:] = position / position[3]
    
    print(pointCloud[int(avev/count), int(aveu/count)])
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
    cv2.waitKey(100)

    return x_world

def getBlockPos(blockId, images):
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
    cv2.waitKey(100)

    # print(x_world)
    return x_world

step = 0
while 1:
    keys = p.getKeyboardEvents()

    # add new blocks on the spot
    if ord('b') in keys:
        state = keys[ord('b')]
        blockIds.append(p.loadURDF("block.urdf", [-0.5, 0, 1]))
        
    if (useRealTimeSimulation):
        dt = datetime.now()
        t = (dt.second / 10.) * 2. * math.pi
    else:
        t = t + 0.1

    if (useSimulation and useRealTimeSimulation == 0):
      p.stepSimulation()

    images = p.getCameraImage(width,
                              height,
                              view_matrix,
                              projection_matrix,
                              shadow=True,
                              renderer=p.ER_BULLET_HARDWARE_OPENGL)
    tpos = getBlockPos(blockIds[0], images)
    tpos1 = getBlockPos2(blockIds[0], images)
    if tpos is None: continue
    if step > 10:
        print("hoge")
        tpos[2] += 0.4

        # pos = [-0.4, 0.2 * math.cos(t), 0.4 + 0.2 * math.sin(t)]
        orn = p.getQuaternionFromEuler([0, -math.pi, 0])
        jointPoses = p.calculateInverseKinematics(kukaId,
                                                  kukaEndEffectorIndex,
                                                  tpos[0:3],
                                                  orn,
                                                  lowerLimits=ll,
                                                  upperLimits=ul,
                                                  jointRanges=jr,
                                                  restPoses=rp)
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
    step += 1
    
    # ls = p.getLinkState(kukaId, kukaEndEffectorIndex)
    # if (hasPrevPose):
    #     p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
    #     p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
    # prevPose = pos
    # prevPose1 = ls[4]
    # hasPrevPose = 1
    
    ############################################3

