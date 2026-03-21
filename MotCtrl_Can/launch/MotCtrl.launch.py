from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    kp_mit = LaunchConfiguration("kp_mit")
    kd_mit = LaunchConfiguration("kd_mit")
    cmd_timeout_ms = LaunchConfiguration("cmd_timeout_ms")
    watchdog_period_ms = LaunchConfiguration("watchdog_period_ms")
    control_period_ms = LaunchConfiguration("control_period_ms")

    node_MotCtrl = Node(
        package="MotCtrl_Can",
        executable="MotCtrl_node",
        name="MotCtrl_node",
        output="screen",
        parameters=[{
            "kp_mit": kp_mit,
            "kd_mit": kd_mit,
            "cmd_timeout_ms": cmd_timeout_ms,
            "watchdog_period_ms": watchdog_period_ms,
            "control_period_ms": control_period_ms,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("kp_mit", default_value="0.0"),
        DeclareLaunchArgument("kd_mit", default_value="0.0"),
        DeclareLaunchArgument("cmd_timeout_ms", default_value="300"),
        DeclareLaunchArgument("watchdog_period_ms", default_value="20"),
        DeclareLaunchArgument("control_period_ms", default_value="5"),
        node_MotCtrl,
    ])
