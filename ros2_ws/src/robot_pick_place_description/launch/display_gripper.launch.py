from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare("robot_pick_place_description")
    xacro_file = PathJoinSubstitution(
        [pkg_share, "urdf", "onrobot_gripper.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [pkg_share, "config", "gripper.rviz"]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro", " ", xacro_file]),
            value_type=str
        )
    }

    return LaunchDescription([
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui"
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[robot_description]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config]
        )
    ])
