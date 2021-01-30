import pybullet as p
import time
import pybullet_data

# 参照した情報
# pybullet Quick start guideのIntroduction script

# connect to server 
physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
time.sleep(3)

p.setRealTimeSimulation(1)

# use the PyBullet_data package
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

# load plane
planeId = p.loadURDF("plane.urdf")

# load frame
frame1_orn = p.getQuaternionFromEuler([0,0,0])
frame1_pos = [1,1,1]
frame1 = p.loadURDF("./data/frame.urdf",
                      basePosition=frame1_pos,
                      baseOrientation=frame1_orn,
                      useFixedBase=1)

frame2_orn = p.getQuaternionFromEuler([0,0,1.57])
frame2_pos = [1,2,1]
frame2 = p.loadURDF("./data/frame.urdf",
                      basePosition=frame2_pos,
                      baseOrientation=frame2_orn,
                      useFixedBase=1)

while p.isConnected(physicsClient):
    pass

p.disconnect()
