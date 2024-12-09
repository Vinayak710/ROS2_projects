#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import numpy as np
from math import atan, atan2, pi, sin, cos, sqrt
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.msg import Trilateration, Landmark

class TrilaterationNode(Node):
    def __init__(self):
        super().__init__('trilateration_node')
        
        # Initialize pose and landmarks
        self.pose = [0.0, 0.0, 0.0]
        
        self.landmarkA = [ 6,  -7]
        self.varA = 0.1
        self.landmarkB = [-8, -7]
        self.varB = 0.1
        self.landmarkC = [ 7, 9]
        self.varC = 0.1
        
        # Create subscribers
        self.odom_sub = self.create_subscription(Odometry,'/odom',self.callback,10)
            
        # Uncomment if using vicon
        # self.vicon_sub = self.create_subscription(
        #     TransformStamped,
        #     '/vicon/tb3_3/tb3_3',
        #     self.callback_vicon,
        #     10)
        
        # Create publisher
        self.trilateration_pub = self.create_publisher(Trilateration,'trilateration_data',10)
            
        # Create timer for publishing
        self.timer = self.create_timer(0.1, self.publish_trilateration)
        
        self.get_logger().info('Trilateration node initialized')

    def heading_from_quaternion(self, x, y, z, w):
        ang_1 = 2*(w*z + x*y)
        ang_2 = 1-2*(y**2 + z**2)
        return atan2(ang_1, ang_2) % (2*pi)

    def callback(self, data):
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        self.pose = [data.pose.pose.position.x,data.pose.pose.position.y,euler_from_quaternion([x,y,z,w])[2]]
    # def callback_vicon(self, data):
    #     pos_x = data.transform.translation.x
    #     pos_y = data.transform.translation.y
    #     orientation_q = data.transform.rotation
    #     heading = self.heading_from_quaternion(
    #         orientation_q.x,
    #         orientation_q.y,
    #         orientation_q.z,
    #         orientation_q.w
    #     )
    #     self.pose = np.array([pos_x, pos_y, heading]).reshape(3,1)
    #     self.get_logger().info(f'PoseFromTrilateration: {self.pose}')

    def dist(self, p1, p2):
        return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**(0.5)

    def publish_trilateration(self):
        # Create landmark messages with noisy distance measurements
        lA = Landmark(
            x=float(self.landmarkA[0]),
            y=float(self.landmarkA[1]),
            distance=float(self.dist(self.pose, self.landmarkA) + 
                         np.random.normal(0, self.varA)),
            variance=float(self.varA)
        )
        print(lA)
        lB = Landmark(
            x=float(self.landmarkB[0]),
            y=float(self.landmarkB[1]),
            distance=float(self.dist(self.pose, self.landmarkB) + 
                         np.random.normal(0, self.varB)),
            variance=float(self.varB)
        )
        
        lC = Landmark(
            x=float(self.landmarkC[0]),
            y=float(self.landmarkC[1]),
            distance=float(self.dist(self.pose, self.landmarkC) + 
                         np.random.normal(0, self.varC)),
            variance=float(self.varC)
        )
        
        # Create and publish trilateration message
        msg = Trilateration()
        msg.landmark_a = lA
        msg.landmark_b = lB
        msg.landmark_c = lC
        
        self.trilateration_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    trilateration_node = TrilaterationNode()
    rclpy.spin(trilateration_node)
    trilateration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()