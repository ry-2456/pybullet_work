# coding: utf-8
import os
import pybullet as p
import time
import pybullet_data

##########  参照した情報 ##########
# Real-Time Physics Simulation Forum : https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12687
    
if __name__ == "__main__":
    print("###########################")
    cid = p.connect(p.SHARED_MEMORY)
    if (cid < 0):
      p.connect(p.GUI)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())  
    p.setGravity(0,0,-10.)

    p.loadURDF("plane.urdf")
    kuka = p.loadURDF("kuka_iiwa/model.urdf")

    string_pos = [0.0, 0.5, 0.5]
    string_orn = p.getQuaternionFromEuler([0, 0, 0])
    string_model_path = "./data/urdf/string_like_object_3.sdf"
    (stringId, ) = p.loadSDF(string_model_path)
    # string_model_path = "./data/urdf/string_like_object.urdf"
    # stringId = p.loadURDF(string_model_path)# , useFixedBase=1)
    p.resetBasePositionAndOrientation(stringId, string_pos, string_orn)

    p.setRealTimeSimulation(1)   # eable real-tiem simulation.
    while p.isConnected():
        pass
