from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #sensor Node
        Node(
            package='robocon_vision_pkg',
            executable='camera_publisher',
            name='camera_publisher',
            parameters=[{'publish_rate': 10.0}]
        ),

        #rocessor Node
        Node(
            package='robocon_vision_pkg',
            executable='zone_detector',
            name='zone_detector'
        )
    ])
