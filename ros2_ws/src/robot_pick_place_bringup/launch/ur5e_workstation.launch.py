from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    world_path = "/home/jesvin/robot_pick_place_project/worlds/workstation_world.sdf"
    models_path = "/home/jesvin/robot_pick_place_project/models"

    ur_launch_path = os.path.join(
        "/home/jesvin/robot_pick_place_project/ros2_ws/Universal_Robots_ROS2_GZ_Simulation",
        "ur_simulation_gz",
        "launch",
        "ur_sim_moveit.launch.py"
    )

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            models_path,
            ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")
        ],
    )

    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_launch_path),
        launch_arguments={
            "ur_type": "ur5e",
            "world_file": world_path,
            "launch_rviz": "true",
        }.items(),
    )

    return LaunchDescription([
        gz_resource_path,
        ur_sim,
    ])

