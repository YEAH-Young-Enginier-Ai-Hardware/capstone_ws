from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 실내용 RKO-LIO 3D global map(.ply)
    indoor_rko_global_map_path = LaunchConfiguration("indoor_rko_global_map_path")

    # 실내용 Nav2 2D map.yaml
    indoor_map_yaml = LaunchConfiguration("indoor_map_yaml")

    # 기존 전체 런치 파일
    # 여기 파일명은 현재 네가 쓰는 Nav2 통합 런치 파일 이름과 같아야 함.
    original_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("yeah_bringup"),
                    "launch",
                    "go2_ouster_rko_lio_nav2.launch.py",
                ]
            )
        ),
        launch_arguments={
            # RKO-LIO localization용 3D map만 실내용으로 교체
            "rko_global_map_path": indoor_rko_global_map_path,

            # Nav2 global costmap / map_server용 2D map만 실내용으로 교체
            "map_yaml": indoor_map_yaml,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "indoor_rko_global_map_path",
                description="Indoor 3D global map path for RKO-LIO localization",
            ),
            DeclareLaunchArgument(
                "indoor_map_yaml",
                description="Indoor 2D map.yaml path for Nav2 map_server",
            ),
            original_nav2_launch,
        ]
    )
