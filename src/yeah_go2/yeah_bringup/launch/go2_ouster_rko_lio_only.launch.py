from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
            go2_ouster_launch,
            TimerAction(period=rko_start_delay_sec, actions=[rko_lio_launch]),
        ]
    )
