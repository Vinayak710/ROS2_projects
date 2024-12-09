#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
import math
from math import atan, atan2, pi, sin, cos, sqrt
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# Assuming you've created these custom messages in ROS2
from my_robot_interfaces.msg import Trilateration

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')
        
        # Define constants
        self.K_samp = 0.1  # sampling time/gain in sec
        self.FILTER_ORDER = 5
        self.varA = 0.1
        self.varB = 0.1
        self.varC = 0.1
        
        # Initialize system matrices
        self.Q = np.eye(3) 
        self.R = np.diag([1,1,1])
        self.P_n_minus_1 = np.eye(3)  # State Covariance
        self.F = np.eye(3)  # System matrix
        self.H = np.eye(3)  #which is used to convert the predicted state estimate

        # Initialize state variables
        self.estimated_pose_n_minus_1= np.array([[0],[0],[0]])
        self.current_measured_pose= np.array([[0],[0],[0]])
        self.noisy_pose = np.array([[0],[0],[0]])
        self.our_predicted_pose = np.array([[0],[0],[0]])
        self.input_sys = np.zeros((2, 1))
        self.last_callback_time = None
        
        # Initialize filters
        self.filter_a = [0] * self.FILTER_ORDER
        self.filter_b = [0] * self.FILTER_ORDER
        self.filter_c = [0] * self.FILTER_ORDER
        self.idxA = self.idxB = self.idxC = 0
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom2_pub = self.create_publisher(String, '/odom2', 10)
        self.waypoint_pub = self.create_publisher(String, '/bot_0/waypoint', 10)
        self.estimatedpose_pub = self.create_publisher(String, '/bot_0/estimatedpose', 10)
        
        # Create subscribers
        self.create_subscription(
            Trilateration,
            '/trilateration_data',
            self.trilateration_callback,
            10)
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        # self.create_subscription(
        #     TransformStamped,
        #     '/vicon/tb3_3/tb3_3',
        #     self.callback_vicon
        # )
        # Create timer for control loop
        self.timer = self.create_timer(0.2, self.control_loop_callback)  # 5Hz
        self.control_timer = 0.0
        
    # def heading_from_quaternion(self, x, y, z, w):
    #     ang_1 = 2*(w*z + x*y)
    #     ang_2 = 1-2*(y**2 + z**2)
    #     return atan2(ang_1, ang_2) % (2*pi)
        
        
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
    #     self.current_measured_pose = np.array([pos_x, pos_y, heading]).reshape(3,1)
  
    def predict_state(self, estimated_pose_n_minus_1):
        self.input_sys.shape = (2, 1)
        self.G = np.array([
            [self.K_samp * cos(float(estimated_pose_n_minus_1[2,0])), 0],
            [self.K_samp * sin(float(estimated_pose_n_minus_1[2,0])), 0],
            [0.0, self.K_samp]
        ])
        estimated_pose_n = (self.F @ estimated_pose_n_minus_1) + (self.G @ self.input_sys)
        return estimated_pose_n
        
    def predict_measurement(self, landmark_a, landmark_b, landmark_c,current_pose):
        d1= landmark_a.distance
        d2= landmark_b.distance
        d3= landmark_c.distance
        x= (d2**2-d1**2)/28 - 1
        y= (30*d1**2 - 28*d3**2 - 2*d2**2 + 1316)/896
        theta= current_pose[2,0]
        measurement = np.array([[x],[y],[theta]])
        return measurement
        
    def odom_callback(self, msg):
        varX = 0.1
        varY = 0.1
        varTHETA = 0.1
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        noise = [
            np.random.normal(0, varX),
            np.random.normal(0, varY),
            np.random.normal(0, varTHETA)
        ]
        self.noisy_pose = np.array([
            msg.pose.pose.position.x + noise[0],
            msg.pose.pose.position.y + noise[1],
            euler_from_quaternion([x, y, z, w])[2] + noise[2]
        ]).reshape(3, 1)
        self.current_measured_pose = self.noisy_pose
            
    def get_waypoint(self, timer):
        X_ref = 4*sin(timer + 1)
        Y_ref = 4*sin(timer)
        self.waypoint_pub.publish(String(data=str((X_ref,Y_ref))))
        return X_ref, Y_ref
        

    def proportional_controller(self, X_ref, Y_ref, noisy_pose):
        # Controller gains
        Kxp = 0.1  # Proportional gain for linear velocity
        Kthetap = 0.2  # Proportional gain for angular velocity

        # Calculate position error
        error_x = X_ref - noisy_pose[0, 0]
        error_y = Y_ref - noisy_pose[1, 0]

        # Compute distance to the target
        distance_error = math.sqrt(error_x**2 + error_y**2)

        # Compute desired angle to the target
        desired_angle = math.atan2(error_y, error_x)

        # Calculate angular error
        angular_error = desired_angle - noisy_pose[2, 0]

        # Normalize angular error to [-pi, pi]
        angular_error = (angular_error + math.pi) % (2 * math.pi) - math.pi

        # Compute control inputs
        linear_velocity = Kxp * distance_error
        angular_velocity = Kthetap * angular_error

        # Optional: Limit control inputs
        max_linear_velocity = 1.0  # Maximum allowed linear velocity
        max_angular_velocity = 1.0  # Maximum allowed angular velocity

        linear_velocity = max(min(linear_velocity, max_linear_velocity), -max_linear_velocity)
        angular_velocity = max(min(angular_velocity, max_angular_velocity), -max_angular_velocity)

        # Update system input
        self.input_sys[0] = linear_velocity  # Linear velocity
        self.input_sys[1] = angular_velocity  # Angular velocity


        
    def trilateration_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.last_callback_time is None or \
        (current_time - self.last_callback_time) >= self.K_samp:
            
            # # Filter measurements
            # self.filter_a[self.idxA] = msg.landmark_a.distance
            # self.filter_b[self.idxB] = msg.landmark_b.distance
            # self.filter_c[self.idxC] = msg.landmark_c.distance
            
            # # Update indices
            # self.idxA = (self.idxA + 1) % self.FILTER_ORDER
            # self.idxB = (self.idxB + 1) % self.FILTER_ORDER
            # self.idxC = (self.idxC + 1) % self.FILTER_ORDER
            
            # # Calculate filtered measurements
            # Y1 = sum(self.filter_a) / self.FILTER_ORDER
            # Y2 = sum(self.filter_b) / self.FILTER_ORDER
            # Y3 = sum(self.filter_c) / self.FILTER_ORDER
            # Y_measured = np.array([[Y1], [Y2], [Y3]])
            
            # EKF Prediction
            self.estimated_pose_n = self.predict_state(self.estimated_pose_n_minus_1)
            
            # Covariance update
            self.P_n = (self.F @ self.P_n_minus_1 @ (self.F.T)) + self.Q
            
            current_pose = self.current_measured_pose
            # Measurement prediction and residual only used because simulating in gasibo while vicon usage you will directly get pose of robor 
            #use it directly as (measurement_position_using_vicon_sensor)
            measurement_position_using_vicon_sensor = self.predict_measurement(
                msg.landmark_a,
                msg.landmark_b,
                msg.landmark_c,
                current_pose
            )

            measurement_residual_y_k = measurement_position_using_vicon_sensor - self.estimated_pose_n

            # Calculate the measurement residual covariance
            S = self.H @ self.P_n @ self.H .T + self.R

            #Calculate the near-optimal Kalman gain
            K_k = self.P_n @ self.H.T @ np.linalg.inv(S)

            # Calculate an updated state estimate for time k
            self.estimated_pose_n = self.estimated_pose_n + (K_k @ measurement_residual_y_k)
            
            # Update the state covariance estimate for time k
            self.P_n_minus_1 = self.P_n - (K_k @ self.H @ self.P_n)
            self.estimated_pose_n_minus_1 = self.estimated_pose_n
            print(self.estimated_pose_n_minus_1)
            # Publish estimated pose
            self.odom2_pub.publish(String(data=f"({self.estimated_pose_n[0,0]}, {self.estimated_pose_n[1,0]})"))
            self.last_callback_time = current_time
        
    def control_loop_callback(self):
        if self.estimated_pose_n_minus_1 is not None:
            X_ref, Y_ref = self.get_waypoint(self.control_timer)
            self.proportional_controller(X_ref, Y_ref, self.estimated_pose_n_minus_1)
            
            velocity_msg = Twist()
            velocity_msg.linear.x = float(self.input_sys[0][0])
            velocity_msg.angular.z = float(self.input_sys[1][0])

            self.control_timer += 0.004 * pi
            self.cmd_vel_pub.publish(velocity_msg)
            # Publish estimated pose as string
            if self.noisy_pose is not None:
                self.estimatedpose_pub.publish(String(data=f"({self.noisy_pose[0, 0]}, {self.noisy_pose[1, 0]})"))

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()