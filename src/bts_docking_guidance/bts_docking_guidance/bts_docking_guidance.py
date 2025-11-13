import sys
import numpy as np
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from bts_docking_guidance.angle_transformation import euler_ned_to_quat, quat_to_euler_ned
from typing import Tuple

class BTSDockingGuidance(Node):
    def __init__(self):
        super().__init__('bts_docking_guidance')

        # Current position
        self.current_position = np.zeros(3)
        self.current_velocity = np.zeros(3)
        self.current_yaw = 0.0

        # Waypoint position
        self.waypoint_position = np.zeros(3)
        self.waypoint_yaw = 0.0
        self.waypoints = []
        self.waypoint_idx = 0

        # Subscriber
        self.create_subscription(
            Odometry,
            "/blueye/aruco_estimation", 
            self.odometry_callback,
            1
        )

        # Publisher
        self.guidance_publisher = self.create_publisher(
            Odometry,
            "/blueye/desired_state",
            1
        )

        # Insert waypoints
        self.generate_waypoints()

        self.get_logger().info("BTSDockingGuidance node has been started.")

    def generate_waypoints(self):
        """
        BTS Tutorial 2 - Define docking waypoints 
        """
        # Define waypoints to reach the docking station, [X, Y, Z, Yaw, sphere_of_acceptance]
        self.waypoints = np.array([
            [3.0, 0.0, -0.3, math.pi, 0.3],
            [2.75, 0.0, -0.3, math.pi, 0.3],
            [2.5, 0.0, -0.3, math.pi, 0.3],
            [2.25, 0.0, -0.3, math.pi, 0.3],
            [2.0, 0.0, -0.3, math.pi, 0.3],
            [1.75, 0.0, -0.3, math.pi, 0.3],
            [1.5, 0.0, -0.3, math.pi, 0.3],
            [1.25, 0.0, -0.3, math.pi, 0.3],
            [1.0, 0.0, -0.3, math.pi, 0.2],
            [0.7, 0.0, -0.1, math.pi, 0.2],
            [0.4, 0.0, -0.05, math.pi, 0.1],
        ])
        """
        BTS Tutorial 2 - End of docking waypoints definition 
        """

    def odometry_callback(self, msg: Odometry):

        # Exit if no waypoints
        if (len(self.waypoints) == 0):
            self.get_logger().warning("No waypoints defined. Exiting..")
            sys.exit(1)
        
        
        # Update current position
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        self.current_position[2] = msg.pose.pose.position.z

        # Update current yaw 
        current_quat = (
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        )
        _, _, self.current_yaw = quat_to_euler_ned(current_quat)

        # Check if we are close to the current waypoint
        if self.waypoint_idx < len(self.waypoints):
            waypoint = self.waypoints[self.waypoint_idx]
            waypoint_pos = waypoint[0:3]
            waypoint_yaw = waypoint[3]
            sphere_of_acceptance = waypoint[4]

            distance_to_waypoint = np.linalg.norm(self.current_position - waypoint_pos)

            if distance_to_waypoint < sphere_of_acceptance:
                self.get_logger().info(f"Reached waypoint {self.waypoint_idx}: {waypoint_pos}")
                self.waypoint_idx += 1
                waypoint = self.waypoints[self.waypoint_idx] if self.waypoint_idx < len(self.waypoints) else None
                if waypoint is not None:
                    self.get_logger().info(f"Next waypoint {self.waypoint_idx}: {waypoint[0:3]}")
                else:
                    self.get_logger().info("All waypoints reached.")
                return 

            # Publish guidance to current waypoint 
            desired_msg = Odometry()
            desired_msg.pose.pose.position.x = waypoint_pos[0]
            desired_msg.pose.pose.position.y = waypoint_pos[1]
            desired_msg.pose.pose.position.z = waypoint_pos[2]
            desired_quat = euler_ned_to_quat(0.0, 0.0, waypoint_yaw)
            desired_msg.pose.pose.orientation.w = desired_quat[0]
            desired_msg.pose.pose.orientation.x = desired_quat[1]
            desired_msg.pose.pose.orientation.y = desired_quat[2]
            desired_msg.pose.pose.orientation.z = desired_quat[3]
            self.guidance_publisher.publish(desired_msg)

        else:
            # Send final waypoint
            waypoint = self.waypoints[-1]
            desired_msg = Odometry()
            desired_msg.pose.pose.position.x = waypoint[0]
            desired_msg.pose.pose.position.y = waypoint[1]
            desired_msg.pose.pose.position.z = waypoint[2]
            desired_quat = euler_ned_to_quat(0.0, 0.0, waypoint[3])
            desired_msg.pose.pose.orientation.w = desired_quat[0]
            desired_msg.pose.pose.orientation.x = desired_quat[1]
            desired_msg.pose.pose.orientation.y = desired_quat[2]
            desired_msg.pose.pose.orientation.z = desired_quat[3]
            self.guidance_publisher.publish(desired_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BTSDockingGuidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()