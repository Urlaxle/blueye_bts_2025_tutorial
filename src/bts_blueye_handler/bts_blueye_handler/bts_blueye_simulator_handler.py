import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64
import numpy as np

class BTSBlueyeSimulatorHandler(Node):
    
    def __init__(self):
        super().__init__('bts_blueye_simulator_handler')
        self.get_logger().info("Initializing BTS Blueye Simulator Handler Node")

        # Subscribe to Blueye Commands
        self.create_subscription(
            WrenchStamped,
            'blueye/commands',
            self.blueye_command_callback,
            1
        )

        # Declare publishers for simulated feedback
        self.surge_left_pub = self.create_publisher(Float64, '/model/blueye/joint/blueye_thruster_1_joint/cmd_thrust', 1)
        self.surge_right_pub = self.create_publisher(Float64, '/model/blueye/joint/blueye_thruster_2_joint/cmd_thrust', 1)
        self.sway_pub = self.create_publisher(Float64, '/model/blueye/joint/blueye_thruster_3_joint/cmd_thrust', 1)
        self.heave_pub = self.create_publisher(Float64, '/model/blueye/joint/blueye_thruster_4_joint/cmd_thrust', 1)

        self.get_logger().info("BTS Blueye Simulator Handler Node Ready")

    def blueye_command_callback(self, msg: WrenchStamped):
        # Log incoming commands for simulation purposes
        surge = max(-1, min(1, msg.wrench.force.x))
        sway = max(-1, min(1, msg.wrench.force.y))
        heave = max(-1, min(1, msg.wrench.force.z))
        yaw = max(-1, min(1, msg.wrench.torque.z))


        # Calculate thruster outputs
        surge_left = Float64()
        surge_right = Float64()
        sway_front = Float64()
        heave_top = Float64()

        # Publish simulated thruster commands 
        surge_left.data = surge * 10.0 + yaw * 10.0
        surge_right.data = surge * 10.0 - yaw * 10.0
        sway_front.data = -sway * 10.0
        heave_top.data = -heave * 10.0
        self.surge_left_pub.publish(surge_left)
        self.surge_right_pub.publish(surge_right)
        self.sway_pub.publish(sway_front)
        self.heave_pub.publish(heave_top)

def main():
    rclpy.init()
    node = BTSBlueyeSimulatorHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()