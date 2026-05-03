from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_period_ms = LaunchConfiguration("control_period_ms")
    watchdog_period_ms = LaunchConfiguration("watchdog_period_ms")
    cmd_timeout_ms = LaunchConfiguration("cmd_timeout_ms")
    feedback_log_period_ms = LaunchConfiguration("feedback_log_period_ms")
    verbose_log = LaunchConfiguration("verbose_log")

    left_leg_dof = LaunchConfiguration("left_leg_dof")
    right_leg_dof = LaunchConfiguration("right_leg_dof")
    left_arm_dof = LaunchConfiguration("left_arm_dof")
    right_arm_dof = LaunchConfiguration("right_arm_dof")

    nominal_joint_pos = LaunchConfiguration("nominal_joint_pos")
    cmd_vel_to_joint_vel_gain = LaunchConfiguration("cmd_vel_to_joint_vel_gain")
    yaw_to_joint_vel_gain = LaunchConfiguration("yaw_to_joint_vel_gain")

    robot_control_node = Node(
        package="robot_control",
        executable="robot_control",
        name="robot_control",
        output="screen",
        parameters=[{
            "control_period_ms": control_period_ms,
            "watchdog_period_ms": watchdog_period_ms,
            "cmd_timeout_ms": cmd_timeout_ms,
            "feedback_log_period_ms": feedback_log_period_ms,
            "verbose_log": verbose_log,
            "left_leg_dof": left_leg_dof,
            "right_leg_dof": right_leg_dof,
            "left_arm_dof": left_arm_dof,
            "right_arm_dof": right_arm_dof,
            "nominal_joint_pos": nominal_joint_pos,
            "cmd_vel_to_joint_vel_gain": cmd_vel_to_joint_vel_gain,
            "yaw_to_joint_vel_gain": yaw_to_joint_vel_gain,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("control_period_ms", default_value="20"),
        DeclareLaunchArgument("watchdog_period_ms", default_value="20"),
        DeclareLaunchArgument("cmd_timeout_ms", default_value="300"),
        DeclareLaunchArgument("feedback_log_period_ms", default_value="1000"),
        DeclareLaunchArgument("verbose_log", default_value="false"),
        DeclareLaunchArgument("left_leg_dof", default_value="3"),
        DeclareLaunchArgument("right_leg_dof", default_value="3"),
        DeclareLaunchArgument("left_arm_dof", default_value="3"),
        DeclareLaunchArgument("right_arm_dof", default_value="3"),
        DeclareLaunchArgument("nominal_joint_pos", default_value="0.0"),
        DeclareLaunchArgument("cmd_vel_to_joint_vel_gain", default_value="1.0"),
        DeclareLaunchArgument("yaw_to_joint_vel_gain", default_value="0.5"),
        robot_control_node,
    ])
