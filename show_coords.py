# coding: utf-8
import pybullet as p
import time
import pybullet_data

##########  参照した情報 ##########

##### sample program #####
# debuDrawItems.py

##### Pybullet Quickstart Guide #####
# p21: Controlling a robot > Base, Joints, Links
# p74: addUserDebugLine 
# 
    
if __name__ == "__main__":


    cid = p.connect(p.SHARED_MEMORY)
    if (cid < 0):
      p.connect(p.GUI)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())  
    p.loadURDF("plane.urdf")
    kuka = p.loadURDF("kuka_iiwa/model.urdf")
    p.addUserDebugText("tip", [0, 0, 0.1],
                       textColorRGB=[1, 0, 0],
                       textSize=1.5,
                       parentObjectUniqueId=kuka,
                       parentLinkIndex=6)

    # 座標を書き入れる

    # p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=kuka, parentLinkIndex=6)
    # p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=kuka, parentLinkIndex=6)
    # p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=kuka, parentLinkIndex=6)

    show_coords(kuka, lineWidth=4., frameLength=1) # show kuka base link
    show_coords(kuka, parentLinkIndex=6, lineWidth=4, frameLength=1) # show kuka tip link
    print("#############################")
    print("The number of kuka joints")
    print(p.getNumJoints(kuka))
    print("#############################")

    p.setRealTimeSimulation(0) # disable real-tiem simulation.
    angle = 0
    while (True):
      time.sleep(0.01)
      p.resetJointState(kuka, 2, angle)
      p.resetJointState(kuka, 3, angle)
      angle += 0.01
