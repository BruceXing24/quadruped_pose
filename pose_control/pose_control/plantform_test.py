import pybullet as p
import numpy as np
import Leg
import pose_calculate as pose
import pybullet_data as pd
PI = 3.1415926
import time







def main():



    count = 0
    angle  = 0

    p.connect(p.GUI)
    p.setGravity(0, 0, 0)
    orn = p.getQuaternionFromEuler([ PI / 2, 0, 0])

    p.setAdditionalSearchPath(pd.getDataPath())
    planeID = p.loadURDF("plane.urdf")
    robot = p.loadURDF("src/urdf/mini_cheetah.urdf", [0, 0, 1.5], useMaximalCoordinates=False,
                    flags=p.URDF_USE_IMPLICIT_CYLINDER)


    table = p.loadURDF("src/urdf1/hexapod_v2.urdf", [0, 0, 1], useMaximalCoordinates=False,
            flags=p.URDF_USE_IMPLICIT_CYLINDER,globalScaling=6.0)

    p.changeVisualShape(objectUniqueId=robot, linkIndex=-1, rgbaColor=[1, 1, 0, 1])
    for i in range(4):
        p.changeVisualShape(objectUniqueId=robot, linkIndex=i * 4, rgbaColor=[1, 1, 1, 1])
        p.changeVisualShape(objectUniqueId=robot, linkIndex=i * 4 + 1, rgbaColor=[0.5, 0.5, 0.5, 1])
        p.changeVisualShape(objectUniqueId=robot, linkIndex=i * 4 + 2, rgbaColor=[0.5, 0.5, 0.5, 1])
        p.changeVisualShape(objectUniqueId=robot, linkIndex=i * 4 + 3, rgbaColor=[1, 0, 0, 1])

    p.setGravity(0, 0, -9.8)
    leg = Leg.LegIK()
    leg.positions_control(robot, [0, -PI / 4, PI / 2], [0, -PI / 4, PI / 2], [0, -PI / 4, PI / 2], [0, -PI / 4, PI / 2])
    p.changeDynamics(bodyUniqueId=robot, linkIndex=3, lateralFriction=4.0)
    p.changeDynamics(bodyUniqueId=robot, linkIndex=7, lateralFriction=4.0)
    p.changeDynamics(bodyUniqueId=robot, linkIndex=11, lateralFriction=4.0)
    p.changeDynamics(bodyUniqueId=robot, linkIndex=15, lateralFriction=4.0)
    p.changeDynamics(bodyUniqueId=robot, linkIndex=-1, mass=7.5)

    p.changeDynamics(bodyUniqueId=planeID, linkIndex=-1, lateralFriction=4.0)

    # p.changeDynamics(bodyUniqueId=table, linkIndex=-1,lateralFriction=10.0,rollingFriction=4.0)

    # p.changeDynamics(bodyUniqueId=table, linkIndex=0,lateralFriction=10.0,rollingFriction=4.0)
    # p.changeDynamics(bodyUniqueId=table, linkIndex=1,lateralFriction=10.0,rollingFriction=4.0)

    p.changeDynamics(bodyUniqueId=table, linkIndex=-1, mass = 3.0)

    for i in range(17):
        info = p.changeDynamics(bodyUniqueId=table, linkIndex=i+1,mass = 1.0)
        print(info)

    num  =  p.getNumJoints(bodyUniqueId=table)
    print("number =={}".format(num))
        

    a = 0.002


    while True:
        # p.setJointMotorControl2(bodyUniqueId=table,
        #                         jointIndex=1,
        #                         controlMode=p.POSITION_CONTROL,
        #                         targetPosition=10,
        #                         force=100,
        #                         positionGain=1.,
        #                         velocityGain= 1.
        #                         )
        # p.stepSimulation() 
        # time.sleep(0.02)

        

        base_info=p.getLinkState(objectUniqueId=robot,linkIndex=-1)
        print(base_info)
        p.setJointMotorControl2(bodyUniqueId=table,
                                    jointIndex=0,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=0,
                                    force=100,
                                    positionGain=1.,
                                    velocityGain= 1.
                                    )
        p.setJointMotorControl2(bodyUniqueId=table,
                            jointIndex=1,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=angle,
                            force=100,
                            positionGain=0.2,
                            velocityGain= 0.2
                            )
        p.setJointMotorControl2(bodyUniqueId=table,
                            jointIndex=4,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=angle,
                            force=1000,
                            positionGain=0.2,
                            velocityGain= 0.2,
                    )
        p.setJointMotorControl2(bodyUniqueId=table,
                            jointIndex=10,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=angle,
                            force=1000,
                            positionGain=0.2,
                            velocityGain= 0.2,
            )





        p.stepSimulation() 
        time.sleep(0.02)
    
        
        
        
        print(count)


        if angle>PI/6:
            a = -0.001
        
        elif angle<-PI/6:
            a = 0.001
        
        angle+=a
        print("angle=={}".format(angle))

        count+=1

    # for i in range(16):
    #     info = p.getDynamicsInfo(bodyUniqueId=robot,linkIndex = i)
    #     print("{}{}".format(i,info))

    #



if __name__ == '__main__':
    main()