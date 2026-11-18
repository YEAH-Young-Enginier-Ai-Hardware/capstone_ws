from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="my_safety_pkg",
            executable="speaker_warning_node",
            name="speaker_warning_node",
            output="screen",
            parameters=[
                {
                    "cooldown_sec": 5.0,
                    "helmet_file": "helmet.mp3",
                    "siren_file": "siren.mp3",
                }
            ]
        )
    ])