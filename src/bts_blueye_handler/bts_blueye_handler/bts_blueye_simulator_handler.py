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
        self.surge_left_pub = self.create_publisher(Float64, 'blueye/surge_left', 1)
        self.surge_right_pub = self.create_publisher(Float64, 'blueye/surge_right', 1)
        self.sway_pub = self.create_publisher(Float64, 'blueye/sway_front', 1)
        self.heave_pub = self.create_publisher(Float64, 'blueye/heave_top', 1)

        self.get_logger().info("BTS Blueye Simulator Handler Node Ready")

    def blueye_command_callback(self, msg: WrenchStamped):
        # Log incoming commands for simulation purposes
        surge = max(-1, min(1, msg.wrench.force.x))
        sway = max(-1, min(1, msg.wrench.force.y))
        heave = max(-1, min(1, msg.wrench.force.z))
        yaw = max(-1, min(1, msg.wrench.torque.z))

        # Thrust allocation logic for simulation
        desired_thrust = np.array([surge, sway, heave, yaw])
        thrust_allocation_matirx = np.array([
            [0.5, 0, 0, 0.5],   # Surge Left
            [0.5, 0, 0, -0.5],  # Surge Right
            [0, 1, 0, 0],   # Sway Front
            [0, 0, 1, 0]    # Heave Top
        ])

        # Calculate thruster outputs
        thruster_outputs = np.linalg.pinv(thrust_allocation_matirx).dot(desired_thrust)
        surge_left = Float64()
        surge_right = Float64()
        sway_front = Float64()
        heave_top = Float64()

        # Publish simulated thruster commands 
        surge_left.data = thruster_outputs[0]
        surge_right.data = thruster_outputs[1]
        sway_front.data = thruster_outputs[2]
        heave_top.data = thruster_outputs[3]
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