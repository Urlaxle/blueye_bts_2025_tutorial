import rclpy
from rclpy.node import Node
from pynput import keyboard
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped 
from std_srvs.srv import SetBool

class KeyboardNode(Node):

    def __init__(self):
        super().__init__("bts_keyboard_node")

        # Publisher to Blueye commands
        self.command_publisher_ = self.create_publisher(WrenchStamped, "/blueye/commands", 1)

        # Setup service client
        self.docking_mode_srv = self.create_client(SetBool, 'blueye/enable_docking')
        self.docking_mode = True
        self.docking_mode_req = SetBool.Request()

        # Starting keyboard listener
        self.keyboard = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.keyboard.start()

        # Variables to store keyboard input
        self.surge = 0.0
        self.sway = 0.0
        self.heave = 0.0
        self.yaw = 0.0

        # Starting timer callback
        self.timer = self.create_timer(0.1, self.publish_command)

    def on_press(self, key):

        # Retrive button
        button = key.char if hasattr(key, 'char') else key.name
        print("Pressed button: {}".format(button))

        if button in ["up", "down"]:
            self.surge = 0.4 if button == "up" else -0.4
        elif button in ["left", "right"]:
            self.sway = 0.4 if button == "left" else -0.4 
        elif button in ["w", "s"]:
            self.heave = 0.4 if button == "w" else -0.4 
        elif button in ["a", "d"]:
            self.yaw = 0.4 if button == "a" else -0.4 

        if button == "x":
            # Call Start Dock Service
            self.docking_mode_req.data = self.docking_mode
            self.future = self.docking_mode_srv.call_async(self.docking_mode_req)
            self.future.add_done_callback(self.on_docking_mode_response)


    def on_docking_mode_response(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'SetBool call failed: {e}')
            return

        if resp.success:
            # flip the local state only after success
            self.get_logger().info(f'Docking mode set to {self.docking_mode}: {resp.message}')
            # Set docking mode to opposite 
            if self.docking_mode == True:
                self.docking_mode = False
            elif self.docking_mode == False:
                self.docking_mode = True
        else:
            self.get_logger().warn(f'Failed to set docking mode: {resp.message}')

    def publish_command(self):

        command_msg = WrenchStamped()

        # Filling its fields
        command_msg.wrench.force.x = float(self.surge)
        command_msg.wrench.force.y = float(self.sway)
        command_msg.wrench.force.z = float(self.heave)
        command_msg.wrench.torque.z = float(self.yaw)

        # Publish message
        self.command_publisher_.publish(command_msg)

    def on_release(self, key):
        # Retrive button
        button = key.char if hasattr(key, 'char') else key.name

        # Check for released key
        if button in ["up", "down"]:
            self.surge = 0.0
        elif button in ["left", "right"]:
            self.sway = 0.0
        elif button in ["w", "s"]:
            self.heave = 0.0
        elif button in ["a","d"]:
            self.yaw = 0.0

def main(args=None):
    rclpy.init(args=args)
    keyboard_node = KeyboardNode()
    rclpy.spin(keyboard_node)
    keyboard_node.keyboard.stop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

