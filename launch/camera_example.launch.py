import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_usb_cam = get_package_share_directory('usb_cam')
    calibration_file = os.path.join(
        get_package_share_directory('camera_pkg2'),
        'param',
        'camera_ext.yaml'
    )
    ns = "mg400"
    camera_node = Node(
        package='usb_cam', 
        executable='usb_cam_node_exe', 
        output='screen',
        namespace = ns,
        name = 'usb_cam_node',
        # namespace=ns,
        parameters=[calibration_file]
        )
    ros_image = Node(
        package='camera_pkg2', 
        executable='ros_image', 
        output='screen',
        namespace = ns,
        name = "ros_image"
        )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(camera_node)
    ld.add_action(ros_image)
    return 