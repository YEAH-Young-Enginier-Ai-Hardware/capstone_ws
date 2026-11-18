from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # rosbridge XML launch 경로
    rosbridge_launch = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'
    )

    return LaunchDescription([

        # 1. FastAPI 서버
        ExecuteProcess(
            cmd=[
                'python3', '-m', 'uvicorn',
                'main:app',
                '--host', '0.0.0.0',
                '--port', '8000'
            ],
            cwd=os.path.expanduser('~/safety_web_server'),
            output='screen'
        ),

        # 2. rosbridge websocket
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(rosbridge_launch)
        ),

        # 3. web_video_server
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen'
        ),

        # 4. 이미지 서버
        ExecuteProcess(
            cmd=[
                'python3', '-m', 'http.server', '8090'
            ],
            cwd=os.path.expanduser('~/safety_event_images'),
            output='screen'
        ),

        # 5. 로봇 위치 WebSocket 브릿지
        Node(
            package='my_safety_pkg',
            executable='odom_ws_bridge',
            name='odom_ws_bridge',
            output='screen',
            parameters=[
                {
                    'odom_topic': '/rko_lio/odometry',
                    'publish_hz': 10.0,
                    'websocket_host': '0.0.0.0',
                    'websocket_port': 8765,
                }
            ]
        ),
    ])
