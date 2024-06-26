from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os


def generate_launch_description():

    pkg_witmotion_ros2 = get_package_share_directory("witmotion_ros2")
    witmotion_params_file = os.path.join(pkg_witmotion_ros2, "config", "witmotion.yaml")
              
    witmotion_node=Node(
        package = 'witmotion_ros2',
        executable = 'witmotion_ros2',
        name='witmotion_node',
        parameters = [witmotion_params_file]
    )

    return LaunchDescription(
        [
            witmotion_node,
        ]
    )