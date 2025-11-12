import sys
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import WrenchStamped
import blueye.sdk


class BTSBlueyeHandler(Node):

    def __init__(self):
        super().__init__('bts_blueye_handler')
        self.get_logger().info("Initializing BTS Blueye Handler Node")

        # Initialize Blueye 
        self.connect_to_blueye()

        # Subscribe to Blueye Commands
        self.create_subscription(
            WrenchStamped,
            'blueye/commands',
            self.blueye_command_callback,
            1
        )
        self.get_logger().info("BTS Blueye Handler Node Connected To Blueye")

    def connect_to_blueye(self):
        # Initialize a connection
        self.blueye_drone = blueye.sdk.Drone()
        # Throw error if connection fails
        if not self.blueye_drone:
            self.get_logger().error("Failed to connect to Blueye Drone")
            sys.exit()

    def blueye_command_callback(self, msg: WrenchStamped):
        # Thresholding incoming values
        surge = max(-1, min(1, msg.wrench.force.x))
        sway = max(-1, min(1, msg.wrench.force.y))
        heave = max(-1, min(1, msg.wrench.force.z))
        yaw = max(-1, min(1, msg.wrench.torque.z))

        # Sending commands 
        self.blueye_drone.motion.surge = surge
        self.blueye_drone.motion.sway = sway
        self.blueye_drone.motion.heave = heave
        self.blueye_drone.motion.yaw = yaw

def main():
    rclpy.init()
    node = BTSBlueyeHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()