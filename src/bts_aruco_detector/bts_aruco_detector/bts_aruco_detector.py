import rclpy
from rclpy.node import Node 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
import numpy as np
import cv2
import math
import pyquaternion
import threading 

# Get ArUco board parameters
from bts_aruco_detector.bts_aruco_docking_station import pos_board, id_board

class BTSArucoDetector(Node):

    def __init__(self):
        super().__init__('bts_aruco_detector')
        self.get_logger().info("Initializing BTS Aruco Detector Node")

        # Declare publisher 
        self.aruco_estimation = self.create_publisher(Odometry, "/blueye/aruco_estimation", 1)

        # Declare subscriber to get the current waypoint
        self.current_waypoint_subscriber = self.create_subscription(Odometry, "/blueye/guidance/desired_state", self.current_waypoint_callback, 1)
        self.current_waypoint_x = 0.0
        self.current_waypoint_y = 0.0
        self.current_waypoint_z = 0.0
        self.current_waypoint_psi = 0.0

        # Declare control mode services
        self.docking_mode = False
        self.dock_mode_service = self.create_service(SetBool, "/blueye/dock_mode/viz", self.dock_mode_callback) 

        # Variables to hold position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_psi = 0.0

        # Set a font
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        # Camera parameters
        self.camera_intrinsics = np.array([[1210.8424, 0, 895.1996],
                                             [0, 1212.7456, 511.1417],
                                             [0, 0, 1]])
        self.distortion_coeffs = np.array([-0.1779, 0.0220, 0.0035, -0.0023, 0.0811])

        # Rotation matrix
        rot_matrix_180_x  = np.zeros((3,3), dtype=np.float32)
        rot_matrix_180_x[0,0] = 1.0
        rot_matrix_180_x[1,1] =-1.0
        rot_matrix_180_x[2,2] =-1.0

        # Define the ArUco dictionary and board
        cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_board = cv2.aruco.Board(pos_board, cv2.aruco_dict, id_board)

        # Start image stream callback in a thread 
        self.image_thread = threading.Thread(target=self.image_loop)
        self.image_thread.daemon = True 
        self.image_thread.start()

    def aruco_detection(self, image):

        """
        BTS Tutorial 1 - Find the corners and ids of the ArUco Markers in the image 
        """
        # Convert image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        parameters = cv2.aruco.DetectorParameters()

        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
                gray, 
                cv2.aruco_dict, 
                parameters=parameters
        )

        if False and len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(image, corners) 
        
        '''
        End of BTS Tutorial 1 
        '''
        if len(corners) > 0:
            self.blueye_pose_estimation(corners, ids)

        return image

    def blueye_pose_estimation(self, corners, ids):
        rvec_initial_guess = np.zeros((3, 1), dtype=np.float32)
        tvec_initial_guess = np.zeros((3, 1), dtype=np.float32)

        _, rvec_board, tvec_board = cv2.aruco.estimatePoseBoard(
            corners, ids, self.aruco_board, self.camera_intrinsics, self.distortion_coeffs,
            rvec_initial_guess, tvec_initial_guess
        )

        R_camera_tag = np.matrix(cv2.Rodrigues(rvec_board)[0]) 
        R_tag_camera = R_camera_tag.T  

        t_tag_camera = (-R_tag_camera @ np.array(tvec_board).reshape(-1,1)).flatten()

        roll_camera_board, pitch_camera_board, yaw_camera_board = self.rotation_to_euler_angles(self.rot_matrix_180_x*R_tag_camera)

        yaw_blueye_wrt_tag = -pitch_camera_board 

        pose_blueye_camera_wrt_tag_est = np.array([t_tag_camera[0,0], t_tag_camera[0,1], t_tag_camera[0,2], yaw_blueye_wrt_tag], dtype=float)

        self.current_x = pose_blueye_camera_wrt_tag_est[0]
        self.current_y = pose_blueye_camera_wrt_tag_est[1]
        self.current_z = pose_blueye_camera_wrt_tag_est[2]
        self.current_psi = pose_blueye_camera_wrt_tag_est[3]

        self.publish_estimate(pose_blueye_camera_wrt_tag_est)

    def image_loop(self):
        # GSTreamer pipeline to the Blueye to minimize latency
        gst_pipeline = (
                    "rtspsrc location=rtsp://192.168.1.101:8554/test latency=50 drop-on-latency=true ! "
                    "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
                    "video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
                )

        # Create a VideoCapture object with the GStreamer pipeline 
        cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        # Make sure its opened 
        if not cap.isOpened():
            self.get_logger().error("Unable to open camera stream.")
            self.get_logger().error(f"OpenCV Build Information:\n{cv2.getBuildInformation()}")
            return

        # Run a image loop 
        try:
            while rclpy.ok():
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().error("Unable to fetch frame")
                    break

                output_image = self.aruco_detection(frame)

                if (output_image is None):
                    self.get_logger().warning("No output image from aruco detection")
                    continue
                    
                # OBS: axis is flipped because want to give pose wrt. aruco frame which is drawn in image
                str_position = "Estimated Pose  x=%4.3f  y=%4.3f  z=%4.3f  yaw=%4.3f" % (self.current_x, self.current_y, self.current_z, np.rad2deg(self.current_psi))
                cv2.putText(output_image, str_position, (260, 75), self.font, 2, (255, 0, 255), 2, cv2.LINE_AA)

                # Display Dock Mode On Screen when its run
                if self.dock_mode:
                    h, w = frame.shape[:2]
                    w_dock = int (w / 2) + 100
                    h_dock = h - int (h / 4) 
                    cv2.putText(output_image, "Dock Mode", (w_dock, h_dock), self.font, 4, (0, 0, 255), 2, cv2.LINE_AA)

                if not self.current_waypoint == [0,0,0,0]:
                    str_position = "Current Waypoint  x=%4.3f  y=%4.3f  z=%4.3f  yaw=%4.3f" % (self.current_waypoint_x, self.current_waypoint_y, self.current_waypoint_z, self.current_waypoint_psi)
                    cv2.putText(output_image, str_position, (260, 150), self.font, 2, (0, 255, 255), 2, cv2.LINE_AA)
                    
                #------- Show Image on Screen ------# 
                cv2.imshow('Estimated Pose Image', output_image)
                key = cv2.waitKey(1) & 0xFF
                
                # If q is pressed program is exited 
                if key == ord('q'):
                    # self.out.release()
                    cv2.destroyAllWindows()
                    rclpy.shutdown()

        finally:
            cap.release()
            # out.release()
            cv2.destroyAllWindows()

    def current_waypoint_callback(self, msg):
        self.current_waypoint_x = msg.pose.pose.position.x
        self.current_waypoint_y = msg.pose.pose.position.y
        self.current_waypoint_z = msg.pose.pose.position.z

        # Extract orientation quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Extract yaw angle from the quaternion 
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy**2 + qz**2)
        self.current_waypoint_psi = math.atan2(siny_cosp, cosy_cosp) * (180.0 / np.pi)

    def dock_mode_callback(self, request, response):
        self.dock_mode = request.data
        response.success = True
        return response

    def rotation_to_euler_angles(self, R):
        assert (self.isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def publish_estimate(self, pose_blueye_camera_wrt_tag_est):

        if not np.all(pose_blueye_camera_wrt_tag_est == 0): 
        
            # Publish the estimated pose as an Odometry
            pose_stamped = Odometry()
            
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "bts_docking_station"
            
            pose_stamped.pose.pose.position.x = float(pose_blueye_camera_wrt_tag_est[2])
            pose_stamped.pose.pose.position.y = float(pose_blueye_camera_wrt_tag_est[0])
            pose_stamped.pose.pose.position.z = float(pose_blueye_camera_wrt_tag_est[1])
            
            # Only publish the yaw angle, but needs to be converted to quaternion
            # Note the - pi because the aruco tag is rotated 180 deg around the z-axis of the new rob_loc tag frame*
            # Also this new frame has z upwards and therefore the [0,0,1] in axis of rotation for yaw
            quat = pyquaternion.Quaternion(axis=[0,0,1], radians=(pose_blueye_camera_wrt_tag_est[3] + np.pi)).normalised # 
            
            pose_stamped.pose.pose.orientation.x = quat.x
            pose_stamped.pose.pose.orientation.y = quat.y
            pose_stamped.pose.pose.orientation.z = quat.z
            pose_stamped.pose.pose.orientation.w = quat.w

            # Publish message
            self.pose_estimated_board_stamped_pub_.publish(pose_stamped)

def main():
    rclpy.init()
    node = BTSArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()