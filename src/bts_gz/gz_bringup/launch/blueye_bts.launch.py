import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch import LaunchContext


def generate_launch_description():

    # Get shared directories
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gz_model_path = os.environ.get("GZ_SIM_RESOURCE_PATH")
    
    if gz_model_path is None:
    	raise RuntimeError("GZ_SIM_RESOURCE_PATH environment variable is not set.")


    # Launch the Gazebo World
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PathJoinSubstitution(
                [
                    "models",
                    "worlds",
                    "bts_docking_world.sdf",
                ]
            )
        }.items(),
    )

    # Spawn the Blueye Vehicle
    spawn_blueye_vehicle = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_blueye_vehicle",
        arguments=[
            "-file", PathJoinSubstitution(["models", "dynamic_assets", "blueye", "blueye.sdf"]),
            "-x", "1.29",
            "-y", "0.17",
            "-z", "-96.62",
            ],
            output="screen",
    )


    # Generate a bridge between ROS2 and Gazebo topics
    ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/model/blueye/joint/blueye_thruster_1_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double",
            "/model/blueye/joint/blueye_thruster_2_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double",
            "/model/blueye/joint/blueye_thruster_3_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double",
            "/model/blueye/joint/blueye_thruster_4_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double",
            "/blueye/odometry_enu/gt@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/blueye/camera/front_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/blueye/camera/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],

        output="screen",
    )

    return LaunchDescription(
        [
            gz_sim,
            spawn_blueye_vehicle,
            ros_bridge,
        ]
    )