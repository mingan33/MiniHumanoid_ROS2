##launch file
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    node_MotCtrl=Node(
        package="MotCtrl_Can",
        executable="MotCtrl_node",
        name="MotCtrl_node",
        output="screen",
    )

    return LaunchDescription([
 	node_MotCtrl,
])
