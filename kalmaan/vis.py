#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import ast

class PlottingNode(Node):
    def __init__(self):
        super().__init__('plotting_node')
        
        # Initialize matplotlib
        plt.ion()
        
        # Initialize tracking variables
        self.X_track = []
        self.Y_track = []
        self.waypoint_X = []
        self.waypoint_Y = []
        self.estimated_X = []
        self.estimated_Y = []
        
        # Create subscribers
        self.odom_sub = self.create_subscription(
            String, 
            '/odom2', 
            self.pose_listener, 
            10
        )
        
        self.waypoint_sub = self.create_subscription(
            String, 
            '/bot_0/waypoint', 
            self.waypoint_listener, 
            10
        )
        
        self.estimatedpose_sub = self.create_subscription(
            String, 
            '/bot_0/estimatedpose', 
            self.ep_listener, 
            10
        )
        
        # Create timer for plot updates
        self.timer = self.create_timer(0.1, self.update_plot)
        
        # Setup plot
        self.setup_plot()
        
    def setup_plot(self):
        self.LIM = 14
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_xlim([-self.LIM/2, self.LIM/2])
        self.ax.set_ylim([-self.LIM/2, self.LIM/2])
        self.ax.grid()
        
        # Initialize plot lines
        self.line_trajectory_plot, = self.ax.plot([], [], 'r-', label="Odometry Trajectory")
        self.line_waypoints_plot, = self.ax.plot([], [], 'g^', label="Waypoints", markersize=5)
        self.line_estimated_plot, = self.ax.plot([], [], 'b+', label="Estimated Pose")
        
        self.ax.legend()
        
    def pose_listener(self, data):
        try:
            # Get the position of the robot from the Odometry message
            Odometry = ast.literal_eval(data.data)            
            # Track the position
            self.X_track.append(Odometry[0])
            self.Y_track.append(Odometry[1])
        except Exception as e:
            self.get_logger().error(f"Error parsing Odometry Trajectory: {e}")
        
    def waypoint_listener(self, data):
        try:
            # Parse the waypoint data (assuming it's a list or tuple)
            waypoint = ast.literal_eval(data.data)
            
            # Append waypoint coordinates
            self.waypoint_X.append(waypoint[0])
            self.waypoint_Y.append(waypoint[1])
            
        except Exception as e:
            self.get_logger().error(f"Error parsing waypoint: {e}")
        
    def ep_listener(self, data):
        try:
            # Parse the estimated pose data (assuming it's a list or tuple)
            estimated_pose = ast.literal_eval(data.data)
            
            # Append estimated pose coordinates
            self.estimated_X.append(estimated_pose[0])
            self.estimated_Y.append(estimated_pose[1])
            
        except Exception as e:
            self.get_logger().error(f"Error parsing estimated pose: {e}")
        
    def update_plot(self):
        # Update plot data
        self.line_trajectory_plot.set_data(self.X_track, self.Y_track)
        self.line_waypoints_plot.set_data(self.waypoint_X, self.waypoint_Y)
        self.line_estimated_plot.set_data(self.estimated_X, self.estimated_Y)
        
        # # Dynamically adjust plot limits
        # if self.X_track and self.Y_track:
        #     x_min = min(min(self.X_track), min(self.waypoint_X + self.estimated_X)) - 1
        #     x_max = max(max(self.X_track), max(self.waypoint_X + self.estimated_X)) + 1
        #     y_min = min(min(self.Y_track), min(self.waypoint_Y + self.estimated_Y)) - 1
        #     y_max = max(max(self.Y_track), max(self.waypoint_Y + self.estimated_Y)) + 1
            
        #     self.ax.set_xlim(x_min, x_max)
        #     self.ax.set_ylim(y_min, y_max)
        
        # Update plot elements
        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
    def cleanup(self):
        # Save final plot
        plt.xlabel('X (in meters)')
        plt.ylabel('Y (in meters)')
        plt.title('Robot Trajectory')
        plt.close()

def main(args=None):
    rclpy.init(args=args)
    plotting_node = PlottingNode()
    
    try:
        rclpy.spin(plotting_node)
    except KeyboardInterrupt:
        plotting_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        # Cleanup
        plotting_node.cleanup()
        plotting_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()