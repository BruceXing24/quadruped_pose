# !/usr/bin/env python3

from rclpy.node import Node
import pybullet as p
from sensor_msgs.msg import Joy
import numpy as np
import rclpy
from . import Leg
from . import pose_calculate as pose
from . import PID
import pybullet_data as pd
PI = 3.1415926
from geometry_msgs.msg import Vector3



class Simulator_rpy(Node):
    def __init__(self, name, robot,testbench):
        super().__init__(name)
        self.get_logger().info("pybullet start")
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.rpy_publisher = self.create_publisher(Vector3,"/rpy_angle",10)
        self.vector3 = Vector3()

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.roll_angle = 0.0
        self.pitch_angle = 0.0
        self.yaw_angle = 0.0

        self.create_timer(0.1, self.timer_callback)  # receive joystick frequency
        self.create_timer(0.01, self.pybullet_fre)  # pybullet step frequency

        self.count = 0
        self.leg = Leg.LegIK()  # position control
        self.PD_r = PID.PID(0.1, 0, 0.2)
        self.PD_p = PID.PID(0.05, 0 ,0.1)


        self.robot = robot  # pybullet robot name , here default == robot
        self.table = testbench
        # recovery parameter

        self.k = 0.05
        self.recover_flag = False
        self.selfBalance_flag  = False

        self.table_angle = 0
        self.angle_delta = 0.01 # 0.003
        base_info=p.getLinkState(self.robot,linkIndex=0)
        self.ori2 = p.getEulerFromQuaternion(base_info[5])





    def joy_callback(self, joy: Joy):
        self.pitch = joy.axes[1]
        self.yaw = joy.axes[0]
        self.roll = joy.axes[3]
        if joy.buttons[4] == 1:
            self.recover_flag = True
        if joy.axes[6]==-1:
            self.selfBalance_flag  = False
            self.get_logger().info(f'selfbalance mode close')
        elif joy.axes[6]==1:
            self.selfBalance_flag  = True
            self.get_logger().info(f'selfbalance mode start')

    def recover(self, current_angle,k):
        delta = k * (current_angle - 0)
        if delta > 0 and delta < 0.1:
            delta = 0.1
        elif delta < 0 and delta > -0.1:
            delta = -0.1
        return current_angle - delta
    
    
    
    def timer_callback(self):

        base_info=p.getLinkState(self.robot,linkIndex=0)
        self.ori2 = p.getEulerFromQuaternion(base_info[5])

        # self.get_logger().info(f'ori1=:{[a*180/np.pi for a in self.ori2 ]}')

        # pose_control mode
        if self.selfBalance_flag==False:
            self.get_logger().info(f'roll=:{self.roll_angle},pitch = {self.pitch_angle},yaw = {self.yaw_angle}')
            if self.recover_flag == True:
                self.roll_angle = self.recover(self.roll_angle,self.k)
                self.pitch_angle = self.recover(self.pitch_angle,self.k)
                self.yaw_angle = self.recover(self.yaw_angle,self.k)

            else:
                self.roll_angle += self.roll / 3
                self.pitch_angle += self.pitch / 3
                self.yaw_angle += self.yaw / 3

            if np.abs(self.roll_angle) < 1 and np.abs(self.pitch_angle) < 1 and np.abs(self.yaw_angle) < 1:
                self.recover_flag = False

            self.roll_angle = np.clip(self.roll_angle, -20, 20)
            self.pitch_angle = np.clip(self.pitch_angle, -20, 20)
            self.yaw_angle = np.clip(self.yaw_angle, -20, 20)




        ##   self-balancing model
        elif self.selfBalance_flag==True:
            if self.table_angle>=PI/6:
                self.angle_delta=-self.angle_delta
            
            elif self.table_angle<=-PI/6:
                self.angle_delta=-self.angle_delta

            self.table_angle+=self.angle_delta


            if np.abs(self.ori2[0]+self.ori2[1]) *180/np.pi > 2  :
                self.get_logger().info(f'PD controller start to balance tobot')
                self.roll_angle = self.PD_r.PD_controller(-self.ori2[0]*180/np.pi, self.roll_angle)*1.5
                self.pitch_angle = self.PD_p.PD_controller(self.ori2[1]*180/np.pi,self.pitch_angle)

                self.roll_angle = np.clip(self.roll_angle, -30, 30)
                self.pitch_angle = np.clip(self.pitch_angle, -30, 30)
                self.yaw_angle = np.clip(self.yaw_angle, -30, 30)

        # publish r p y angle to 3-D graph
        self.vector3.x = self.roll_angle
        self.vector3.y = self.pitch_angle
        self.vector3.z = self.yaw_angle
        self.rpy_publisher.publish(self.vector3)





    def pybullet_fre(self):
        if self.count <= 100:
            print("initlizing")

        if self.count >= 300:
            matrix_ABs = pose.get_AB(self.roll_angle, self.pitch_angle, self.yaw_angle).T
            x1, y1, z1 = matrix_ABs[0, 0], matrix_ABs[0, 1], matrix_ABs[0, 2]
            x2, y2, z2 = matrix_ABs[1, 0], matrix_ABs[1, 1], matrix_ABs[1, 2]
            x3, y3, z3 = matrix_ABs[2, 0], matrix_ABs[2, 1], matrix_ABs[2, 2]
            x4, y4, z4 = matrix_ABs[3, 0], matrix_ABs[3, 1], matrix_ABs[3, 2]

            theta1, theta2, theta3 = self.leg.IK_R(x1, y1, -z1)
            theta4, theta5, theta6 = self.leg.IK_L(x2, y2, -z2)
            theta7, theta8, theta9 = self.leg.IK_R(x3, y3, -z3)
            theta10, theta11, theta12 = self.leg.IK_L(x4, y4, -z4)

            # for checking data
            # print("x1,y1,z1=={},{},{}".format(x1,y1,z1))
            # print("x2,y2,z2=={},{},{}".format(x2,y2,z2))
            # print("x3,y3,z3=={},{},{}".format(x3,y3,z3))
            # print("x4,y4,z4=={},{},{}".format(x4,y4,z4))

            self.leg.positions_control(self.robot, [theta1, theta2, theta3], [theta4, theta5, theta6],
                                       [theta7, theta8, theta9],[theta10, theta11, theta12])
            if self.count%3==0:
                self.leg.table_control(self.table,[0, 0, 0,
                                                   self.table_angle,self.table_angle ,self.table_angle])
        self.count += 1
        p.stepSimulation()





def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    # load urdf
    planeID = p.loadURDF("plane.urdf")
    robot = p.loadURDF("src/urdf/mini_cheetah.urdf", [0, 0, 1.4], useMaximalCoordinates=False,flags=p.URDF_USE_IMPLICIT_CYLINDER)
    table = p.loadURDF("src/urdf1/hexapod_v2.urdf", [0, 0, 1.], useMaximalCoordinates=False,flags=p.URDF_USE_IMPLICIT_CYLINDER,globalScaling=6.0)
    p.changeVisualShape(objectUniqueId=robot, linkIndex=-1, rgbaColor=[1, 1, 0, 1])

    # change view of mini cheetah 3
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
    p.changeDynamics(bodyUniqueId=table, linkIndex=-1,lateralFriction=4.0)
    p.changeDynamics(bodyUniqueId=table, linkIndex=-1, mass = 3.0)

    for i in range(17):
        info = p.changeDynamics(bodyUniqueId=table, linkIndex=i+1,mass = 1.0)
    for i in range(19):
        p.changeVisualShape(objectUniqueId=table, linkIndex=i, rgbaColor=[0, 1, 0, 1])
    leg.table_control(table,[0,0,0,0,0,0])


    # joint_info = p.getJointInfo(bodyUniqueId=table,jointIndex=1)
    # print(joint_info)
    # for i in range(3):
    #     info = p.getDynamicsInfo(bodyUniqueId=table, linkIndex=i)
    #     print(info)
    # for i in range(16):
    #     info = p.getDynamicsInfo(bodyUniqueId=robot,linkIndex = i)
    #     print("{}{}".format(i,info))
    rclpy.init()
    node = Simulator_rpy("rpy_controller", robot,table)
    while (1):
        rclpy.spin(node)
        rclpy.shutdown()


if __name__ == '__main__':
    main()

