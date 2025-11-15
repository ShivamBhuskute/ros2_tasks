from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #sensor Node
        Node(
            package='node_comm_first',
            executable='sensor_node',
            name='sensor_node',
            parameters=[{'publish_rate': 5}]
        ),

        #rocessor Node
        Node(
            package='node_comm_first',
            executable='processor_node',
            name='processor_node'
        ),

        #logger Node
        Node(
            package='node_comm_first',
            executable='logger_node',
            name='logger_node'
        )
    ])
