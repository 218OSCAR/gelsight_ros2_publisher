from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gelsight_ros2_publisher',
            executable='gelsight_publisher_node',
            name='gelsight_publisher',
            output='screen',
            parameters=[{
                'config_path': '/home/tailai.cheng/gs_sdk/examples/configs/gsmini.yaml',
                'topic_name': '/gelsight/image_raw'
            }]
        )
    ])
