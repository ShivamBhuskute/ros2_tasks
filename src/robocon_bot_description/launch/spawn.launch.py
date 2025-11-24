from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = FindPackageShare("robocon_bot_description").find("robocon_bot_description")
    world_share = FindPackageShare("robocon_arena").find("robocon_arena")

    world_path = PathJoinSubstitution([world_share, "worlds", "arena.world"])
    urdf_path = PathJoinSubstitution([pkg_share, "urdf", "robocon_bot.urdf.xacro"])

    return LaunchDescription([

        # Launch Gazebo
        ExecuteProcess(
            cmd=["gazebo", "--verbose", world_path],
            output="screen"
        ),

        # Spawn robot
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "robocon_bot",
                "-file", urdf_path,
                "-x", "0", "-y", "0", "-z", "0.3"

            ],
            output="screen"
        )
    ])
