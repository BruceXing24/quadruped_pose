# !/usr/bin/env python3
import rclpy
from rclpy.node import Node
from . import draw
import matplotlib.pyplot as plt
from geometry_msgs.msg import Vector3


class RPY_Controller(Node):
      def __init__(self,name):
            super().__init__(name)
            self.get_logger().info("graph is created")
            self.create_timer(0.1,self.timer_callback)
            self.create_subscription(Vector3,"/rpy_angle",self.rpy_callback, 10)
            self.roll_angle = 0.0
            self.pitch_angle = 0.0
            self.yaw_angle = 0.0
            # fig = plt.figure()
            self.ax = plt.axes(projection='3d')


      def rpy_callback(self,vector:Vector3):
            self.roll_angle = vector.x
            self.pitch_angle = vector.y
            self.yaw_angle = vector.z

      def timer_callback(self):
            draw.show(self.roll_angle,self.pitch_angle,self.yaw_angle,self.ax)



def main():
      rclpy.init()
      node = RPY_Controller("graph_node")
      rclpy.spin(node)
      rclpy.shutdown()      




if __name__=='__main__':
      main()
