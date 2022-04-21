import os
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    params_file = "g29.yaml"
    params = os.path.join(
        get_package_share_directory('ros_g29_force_feedback'),
        "config",
        params_file)
    g29_ff = Node(
            package="ros_g29_force_feedback",
            executable="g29_force_feedback",
            name="g29_force_feedback",
            parameters=[params])
    return LaunchDescription([
        g29_ff
    ])