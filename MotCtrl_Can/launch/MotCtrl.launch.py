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
    can0_start_id = LaunchConfiguration("can0_start_id")
    can1_start_id = LaunchConfiguration("can1_start_id")
    can2_start_id = LaunchConfiguration("can2_start_id")
    can3_start_id = LaunchConfiguration("can3_start_id")
    can0_motor_count = LaunchConfiguration("can0_motor_count")
    can1_motor_count = LaunchConfiguration("can1_motor_count")
    can2_motor_count = LaunchConfiguration("can2_motor_count")
    can3_motor_count = LaunchConfiguration("can3_motor_count")
    can0_joint_dirs = LaunchConfiguration("can0_joint_dirs")
    can1_joint_dirs = LaunchConfiguration("can1_joint_dirs")
    can2_joint_dirs = LaunchConfiguration("can2_joint_dirs")
    can3_joint_dirs = LaunchConfiguration("can3_joint_dirs")
    can0_motor_types = LaunchConfiguration("can0_motor_types")
    can1_motor_types = LaunchConfiguration("can1_motor_types")
    can2_motor_types = LaunchConfiguration("can2_motor_types")
    can3_motor_types = LaunchConfiguration("can3_motor_types")

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
            "can0_start_id": can0_start_id,
            "can1_start_id": can1_start_id,
            "can2_start_id": can2_start_id,
            "can3_start_id": can3_start_id,
            "can0_motor_count": can0_motor_count,
            "can1_motor_count": can1_motor_count,
            "can2_motor_count": can2_motor_count,
            "can3_motor_count": can3_motor_count,
            "can0_joint_dirs": can0_joint_dirs,
            "can1_joint_dirs": can1_joint_dirs,
            "can2_joint_dirs": can2_joint_dirs,
            "can3_joint_dirs": can3_joint_dirs,
            "can0_motor_types": can0_motor_types,
            "can1_motor_types": can1_motor_types,
            "can2_motor_types": can2_motor_types,
            "can3_motor_types": can3_motor_types,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("kp_mit", default_value="0.0"),
        DeclareLaunchArgument("kd_mit", default_value="0.0"),
        DeclareLaunchArgument("cmd_timeout_ms", default_value="300"),
        DeclareLaunchArgument("watchdog_period_ms", default_value="20"),
        DeclareLaunchArgument("control_period_ms", default_value="5"),
        DeclareLaunchArgument("can0_start_id", default_value="0"),
        DeclareLaunchArgument("can1_start_id", default_value="0"),
        DeclareLaunchArgument("can2_start_id", default_value="0"),
        DeclareLaunchArgument("can3_start_id", default_value="0"),
        DeclareLaunchArgument("can0_motor_count", default_value="3"),
        DeclareLaunchArgument("can1_motor_count", default_value="3"),
        DeclareLaunchArgument("can2_motor_count", default_value="3"),
        DeclareLaunchArgument("can3_motor_count", default_value="3"),
        DeclareLaunchArgument("can0_joint_dirs", default_value="[1, -1, 1]"),
        DeclareLaunchArgument("can1_joint_dirs", default_value="[1, -1, 1]"),
        DeclareLaunchArgument("can2_joint_dirs", default_value="[1, -1, 1]"),
        DeclareLaunchArgument("can3_joint_dirs", default_value="[1, -1, 1]"),
        DeclareLaunchArgument("can0_motor_types", default_value="dm,encos,dm"),
        DeclareLaunchArgument("can1_motor_types", default_value="dm,encos,dm"),
        DeclareLaunchArgument("can2_motor_types", default_value="dm,encos,dm"),
        DeclareLaunchArgument("can3_motor_types", default_value="dm,encos,dm"),
        node_MotCtrl,
    ])
