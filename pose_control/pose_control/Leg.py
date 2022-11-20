# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/11/13 17:14

import numpy as np
import pybullet as p


# commit1 :
#     hip2shoulder = 0.06,
#     shoulder2elbow = 0.208,
#     elbow2ground = 0.189,
#     with lateral friciton =4

class LegIK():
    def __init__(self,
                 leg_name="right",
                 hip2shoulder =0.06,
                 shoulder2elbow = 0.208,
                 elbow2ground = 0.189,
                 ):
        self.leg_name = leg_name
        self.l1 = hip2shoulder
        self.l2 = shoulder2elbow
        self.l3 = elbow2ground
        self.theta1_limit = [-np.pi / 2, np.pi / 2]      
        self.theta2_limit = [-np.pi / 4, np.pi / 4]
        self.theta3_limit = [-np.pi / 2, np.pi / 2]
        
        self.Position_Gain = 0.1
        self.Velocity_Gain = 0.1
        self.Max_velocity = 8
        self.force = 10


    def IK(self,x,y,z):
        cos_theta3_rad = (x**2 +z**2 - (self.l2**2+self.l3**2) )/ (2 *self.l2 *self.l3)
        theta3_rad = np.arccos(cos_theta3_rad)

        cos_alpha_rad = ( self.l2**2 + x**2 + z**2- self.l3**2 ) /  ( 2 *self.l2* np.sqrt(x**2+z**2) )
        alpha_rad = np.arccos(cos_alpha_rad)
        theta2_rad = np.pi/2 - alpha_rad - np.arctan2(z , x)

        gamm1 = np.arctan(y/z)
        gamm2 = np.arccos( self.l1/np.sqrt(z**2+y**2) )
        print("gamm1=={},gamm2=={}".format(gamm1/np.pi*180,gamm2/np.pi*180))
        theta1_rad = np.pi/2- gamm1-gamm2
        return [-theta1_rad,theta2_rad,theta3_rad]
        # 错误在于 hip摆动了，高度就会变化，但是theta2,3 都没有考虑theta1 的摆动

    def IK_R(self,x,y,z):
        D=(x**2+y**2+z**2-self.l1**2-self.l2**2-self.l3**2)/(2*self.l2*self.l3)

        theta3 = np.arctan2(-np.sqrt(1-D**2),D)

        theta1 = -np.arctan2(-z,-y)-np.arctan2(np.sqrt(y**2+z**2-self.l1**2),-self.l1)
        theta2 = np.arctan2(x,np.sqrt(y**2+z**2-self.l1**2))-np.arctan2(self.l3*np.sin(theta3),self.l2+self.l3*np.cos(theta3))


        return [theta1,-theta2,-theta3]

    def IK_L(self,x,y,z):
        D=(x**2+y**2+z**2-self.l1**2-self.l2**2-self.l3**2)/(2*self.l2*self.l3)

        theta3 = np.arctan2(-np.sqrt(1-D**2),  D)

        theta1 = -np.arctan2(-z,y)-np.arctan2(np.sqrt(y**2+z**2-self.l1**2),-self.l1)
        theta2 = np.arctan2(x,np.sqrt(y**2+z**2-self.l1**2))-np.arctan2(self.l3*np.sin(theta3),self.l2+self.l3*np.cos(theta3))


        return [-theta1,-theta2,-theta3]



    def positions_control(self,body_name, LF, RF, LH, RH):
        for i in range(3):
            p.setJointMotorControl2(bodyIndex=body_name,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=RF[i],
                                    positionGain=self.Position_Gain,
                                    velocityGain=self.Velocity_Gain,
                                    force=self.force,
                                    maxVelocity=self.Max_velocity
                                    )
            p.setJointMotorControl2(bodyIndex=body_name,
                                    jointIndex=4+i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=LF[i],
                                    positionGain=self.Position_Gain,
                                    velocityGain=self.Velocity_Gain,
                                    force=self.force,
                                    maxVelocity=self.Max_velocity
                                    )
            p.setJointMotorControl2(bodyIndex=body_name,
                                    jointIndex= 8 + i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=RH[i],
                                    positionGain=self.Position_Gain,
                                    velocityGain=self.Velocity_Gain,
                                    force=self.force,
                                    maxVelocity=self.Max_velocity
                                    )
            p.setJointMotorControl2(bodyIndex=body_name,
                                    jointIndex= 12 + i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=LH[i],
                                    positionGain=self.Position_Gain,
                                    velocityGain=self.Velocity_Gain,
                                    force=self.force,
                                    maxVelocity=self.Max_velocity
                                    )

    def table_control(self,bodyname,angle):
        for i in range(6):
            p.setJointMotorControl2(bodyUniqueId=bodyname,
                                    jointIndex=i*3+1,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=angle[i],
                                    force=100,
                                    positionGain=0.2,
                                    velocityGain= 0.2
                                    )


