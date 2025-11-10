from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_example_bringup diff_drive.launch.py',
            executable='diff_drive.launch.py',
        ),

        # Fill in the strings here so that the correct
        # nodes are launched
        Node(
            package='lab4',
            executable='bumper',
        ),
    ])

