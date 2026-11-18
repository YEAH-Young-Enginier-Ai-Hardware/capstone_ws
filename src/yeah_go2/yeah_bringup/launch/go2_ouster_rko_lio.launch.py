from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # ADDED (2026-03-28): inlined go2_ouster_rko args
    go2_ouster_ns = LaunchConfiguration("go2_ouster_ns")
    go2_sensor_hostname = LaunchConfiguration("go2_sensor_hostname")
    go2_udp_dest = LaunchConfiguration("go2_udp_dest")
    go2_lidar_port = LaunchConfiguration("go2_lidar_port")
    go2_imu_port = LaunchConfiguration("go2_imu_port")
    go2_timestamp_mode = LaunchConfiguration("go2_timestamp_mode")
    go2_sensor_frame = LaunchConfiguration("go2_sensor_frame")
    go2_lidar_frame = LaunchConfiguration("go2_lidar_frame")
    go2_imu_frame = LaunchConfiguration("go2_imu_frame")
    go2_point_cloud_frame = LaunchConfiguration("go2_point_cloud_frame")
    go2_pub_static_tf = LaunchConfiguration("go2_pub_static_tf")
    go2_use_rviz = LaunchConfiguration("go2_use_rviz")
    go2_rviz_config = LaunchConfiguration("go2_rviz_config")
    go2_use_imu_bridge = LaunchConfiguration("go2_use_imu_bridge")
    go2_imu_bridge_in_topic = LaunchConfiguration("go2_imu_bridge_in_topic")
    go2_imu_bridge_out_topic = LaunchConfiguration("go2_imu_bridge_out_topic")
    go2_imu_bridge_frame_id = LaunchConfiguration("go2_imu_bridge_frame_id")
    go2_publish_odom_tf = LaunchConfiguration("go2_publish_odom_tf")
    go2_publish_odom_msg = LaunchConfiguration("go2_publish_odom_msg")
    go2_tf_topic = LaunchConfiguration("go2_tf_topic")
    go2_min_range = LaunchConfiguration("go2_min_range")
    go2_static_tf_x = LaunchConfiguration("go2_static_tf_x")
    go2_static_tf_y = LaunchConfiguration("go2_static_tf_y")
    go2_static_tf_z = LaunchConfiguration("go2_static_tf_z")
    go2_static_tf_yaw = LaunchConfiguration("go2_static_tf_yaw")
    go2_static_tf_pitch = LaunchConfiguration("go2_static_tf_pitch")
    go2_static_tf_roll = LaunchConfiguration("go2_static_tf_roll")

    # ADDED (2026-03-28): rko_lio args
    mode = LaunchConfiguration("mode")
    config_file = LaunchConfiguration("config_file")
    rko_rviz = LaunchConfiguration("rko_rviz")
    rko_rviz_config_file = LaunchConfiguration("rko_rviz_config_file")
    global_map_path = LaunchConfiguration("global_map_path")
    imu_topic = LaunchConfiguration("imu_topic")
    imu_frame = LaunchConfiguration("imu_frame")
    lidar_topic = LaunchConfiguration("lidar_topic")
    lidar_frame = LaunchConfiguration("lidar_frame")
    base_frame = LaunchConfiguration("base_frame")
    rko_start_delay_sec = LaunchConfiguration("rko_start_delay_sec")

    # realsense_tf add 0514
    realsense_static_tf_x = LaunchConfiguration("realsense_static_tf_x")
    realsense_static_tf_y = LaunchConfiguration("realsense_static_tf_y")
    realsense_static_tf_z = LaunchConfiguration("realsense_static_tf_z")
    realsense_static_tf_yaw = LaunchConfiguration("realsense_static_tf_yaw")
    realsense_static_tf_pitch = LaunchConfiguration("realsense_static_tf_pitch")
    realsense_static_tf_roll = LaunchConfiguration("realsense_static_tf_roll")

    go2_ouster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("yeah_bringup"), "launch", "go2_ouster.launch.py"]
            )
        ),
        launch_arguments={
            "ouster_ns": go2_ouster_ns,
            "sensor_hostname": go2_sensor_hostname,
            "udp_dest": go2_udp_dest,
            "lidar_port": go2_lidar_port,
            "imu_port": go2_imu_port,
            "timestamp_mode": go2_timestamp_mode,
            "sensor_frame": go2_sensor_frame,
            "lidar_frame": go2_lidar_frame,
            "imu_frame": go2_imu_frame,
            "point_cloud_frame": go2_point_cloud_frame,
            "pub_static_tf": go2_pub_static_tf,
            "use_rviz": go2_use_rviz,
            "rviz_config": go2_rviz_config,
            "use_imu_bridge": go2_use_imu_bridge,
            "imu_bridge_in_topic": go2_imu_bridge_in_topic,
            "imu_bridge_out_topic": go2_imu_bridge_out_topic,
            "imu_bridge_frame_id": go2_imu_bridge_frame_id,
            "publish_odom_tf": go2_publish_odom_tf,
            "publish_odom_msg": go2_publish_odom_msg,
            "tf_topic": go2_tf_topic,
            "min_range": go2_min_range,
            "x": go2_static_tf_x,
            "y": go2_static_tf_y,
            "z": go2_static_tf_z,
            "yaw": go2_static_tf_yaw,
            "pitch": go2_static_tf_pitch,
            "roll": go2_static_tf_roll,
        }.items(),
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={
            "camera_name": "camera",
            "camera_namespace": "camera",
            "publish_tf": "true",
            "tf_publish_rate": "0.0",
            "align_depth.enable": "true",
            "rgb_camera.color_profile": "640x480x15",
            "depth_module.depth_profile": "640x480x15",
            # add 0524
            "pointcloud.enable": "true",
            "pointcloud.ordered_pc": "true",
            "pointcloud.allow_no_texture_points": "true",
        }.items(),
    )

    # add for yolo
    cuda_ld_library_path = SetEnvironmentVariable(
        name="LD_LIBRARY_PATH",
        value="/usr/local/cuda-12.6/targets/aarch64-linux/lib:"
        + os.environ.get("LD_LIBRARY_PATH", ""),
    )

    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("yolo_bringup"), "launch", "yolo.launch.py"]
            )
        ),
        launch_arguments={
            "use_tracking": "False",
            "use_3d": "True",
            "model": "yolov8n-pose.pt",
            "input_image_topic": "/camera/camera/color/image_raw",
            "image_reliability": "1",
            "input_depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "depth_image_reliability": "1",
            "input_depth_info_topic": "/camera/camera/aligned_depth_to_color/camera_info",
            "depth_info_reliability": "1",
            "target_frame": "camera_color_optical_frame",
            "device": "cuda:0",
            "threshold": "0.5",
            "half": "True",
            "fuse_model": "True",
        }.items(),
    )
    # finish for yolo

    # add web launch
    safety_web_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("my_safety_pkg"), "launch", "safety_web_stack.launch.py"]
            )
        )
    )

    # Add realsense TF
    realsense_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_realsense_tf",
        arguments=[
            realsense_static_tf_x,
            realsense_static_tf_y,
            realsense_static_tf_z,
            realsense_static_tf_yaw,
            realsense_static_tf_pitch,
            realsense_static_tf_roll,
            "base_link",
            "camera_link",
        ],
        output="screen",
    )

    fall_detection_node = Node(
        package="my_safety_pkg",
        executable="fall_detection_node",
        name="fall_detection_node",
        output="screen",
    )

    helmet_detection_node = Node(
        package="my_safety_pkg",
        executable="helmet_detection_node",
        name="helmet_detection_node",
        output="screen",
    )

    # Speaker warning node
    # /helmet_warning=True -> helmet.mp3
    # /fall_warning=True   -> siren.mp3
    speaker_warning_node = Node(
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
        ],
    )

    rko_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rko_lio"), "launch", "odometry.launch.py"]
            )
        ),
        launch_arguments={
            "mode": mode,
            "config_file": config_file,
            "rviz": rko_rviz,
            "rviz_config_file": rko_rviz_config_file,
            "global_map_path": global_map_path,
            "imu_topic": imu_topic,
            "imu_frame": imu_frame,
            "lidar_topic": lidar_topic,
            "lidar_frame": lidar_frame,
            "base_frame": base_frame,
        }.items(),
    )

    return LaunchDescription(
        [
            # ADDED (2026-03-28): go2/ouster defaults
            DeclareLaunchArgument("go2_ouster_ns", default_value="ouster"),
            DeclareLaunchArgument("go2_sensor_hostname", description="Ouster sensor hostname or address"),
            DeclareLaunchArgument("go2_udp_dest", default_value=""),
            DeclareLaunchArgument("go2_lidar_port", default_value="7502"),
            DeclareLaunchArgument("go2_imu_port", default_value="7503"),
            DeclareLaunchArgument("go2_timestamp_mode", default_value="TIME_FROM_ROS_TIME"),
            DeclareLaunchArgument("go2_sensor_frame", default_value="os_sensor"),
            DeclareLaunchArgument("go2_lidar_frame", default_value="os_lidar"),
            DeclareLaunchArgument("go2_imu_frame", default_value="os_imu"),
            DeclareLaunchArgument("go2_point_cloud_frame", default_value="os_sensor"),
            DeclareLaunchArgument("go2_pub_static_tf", default_value="true"),
            # ADDED (2026-03-28): keep only the rko_lio RViz by default.
            DeclareLaunchArgument("go2_use_rviz", default_value="false"),
            DeclareLaunchArgument("go2_use_imu_bridge", default_value="true"),
            DeclareLaunchArgument("go2_publish_odom_tf", default_value="false"),
            DeclareLaunchArgument("go2_publish_odom_msg", default_value="true"),
            DeclareLaunchArgument("go2_tf_topic", default_value="/tf_go2"),
            DeclareLaunchArgument("go2_min_range", default_value="0.40"),
            DeclareLaunchArgument("go2_imu_bridge_in_topic", default_value="/imu"),
            DeclareLaunchArgument("go2_imu_bridge_out_topic", default_value="/go2/imu"),
            DeclareLaunchArgument("go2_imu_bridge_frame_id", default_value="imu"),
            DeclareLaunchArgument(
                "go2_rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("yeah_bringup"), "config", "go2_ouster_odom.rviz"]
                ),
            ),
            DeclareLaunchArgument("go2_static_tf_x", default_value="-0.085"),
            DeclareLaunchArgument("go2_static_tf_y", default_value="0.00"),
            DeclareLaunchArgument("go2_static_tf_z", default_value="0.195"),
            DeclareLaunchArgument("go2_static_tf_yaw", default_value="0.0"),
            DeclareLaunchArgument("go2_static_tf_pitch", default_value="0.0"),
            DeclareLaunchArgument("go2_static_tf_roll", default_value="0.0"),
            # ADDED (2026-03-28): rko_lio defaults (your current settings)
            DeclareLaunchArgument("mode", default_value="online"),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("yeah_bringup"), "config", "rko_lio_go2.yaml"]
                ),
            ),
            DeclareLaunchArgument("rko_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rko_rviz_config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("yeah_bringup"), "config", "rko_lio_go2.rviz"]
                ),
            ),
            DeclareLaunchArgument(
                "global_map_path",
                default_value="",
            ),
            DeclareLaunchArgument("imu_topic", default_value="/go2/imu"),
            DeclareLaunchArgument("imu_frame", default_value="imu"),
            DeclareLaunchArgument("lidar_topic", default_value="/ouster/points"),
            DeclareLaunchArgument("lidar_frame", default_value="os_sensor"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("rko_start_delay_sec", default_value="20.0"),

            # add realsense_TF 0514
            DeclareLaunchArgument("realsense_static_tf_x", default_value="0.25"),
            DeclareLaunchArgument("realsense_static_tf_y", default_value="0.00"),
            DeclareLaunchArgument("realsense_static_tf_z", default_value="0.18"),
            DeclareLaunchArgument("realsense_static_tf_yaw", default_value="0.0"),
            DeclareLaunchArgument("realsense_static_tf_pitch", default_value="0.0"),
            DeclareLaunchArgument("realsense_static_tf_roll", default_value="0.0"),

            DeclareLaunchArgument("yolo_start_delay_sec", default_value="5.0"),
            DeclareLaunchArgument("fall_start_delay_sec", default_value="10.0"),
            DeclareLaunchArgument("speaker_start_delay_sec", default_value="12.0"),
            DeclareLaunchArgument("helmet_start_delay_sec", default_value="15.0"),
            DeclareLaunchArgument("safety_web_start_delay_sec", default_value="25.0"),

            go2_ouster_launch,
            cuda_ld_library_path,
            realsense_launch,
            realsense_static_tf,

            # TimerAction(
            #     period=LaunchConfiguration("yolo_start_delay_sec"),
            #     actions=[yolo_launch],
            # ),

            TimerAction(
                period=rko_start_delay_sec,
                actions=[rko_lio_launch],
            ),

            # TimerAction(
            #     period=LaunchConfiguration("fall_start_delay_sec"),
            #     actions=[
            #         fall_detection_node,
            #     ],
            # ),

            # TimerAction(
            #     period=LaunchConfiguration("speaker_start_delay_sec"),
            #     actions=[
            #         speaker_warning_node,
            #     ],
            # ),

            # TimerAction(
            #     period=LaunchConfiguration("helmet_start_delay_sec"),
            #     actions=[
            #         helmet_detection_node,
            #     ],
            # ),

            # TimerAction(
            #     period=LaunchConfiguration("safety_web_start_delay_sec"),
            #     actions=[
            #         safety_web_stack_launch,
            #     ],
            # ),
        ]
    )
