"""Localization: 기준 지도(maps/nav)를 불러와 시작 위치를 스스로 찾고, 그 지도 위에서 위치 추적.

  ros2 launch luco3dnav_bringup localization.launch.py                       # 기준 지도 = maps/nav
  ros2 launch luco3dnav_bringup localization.launch.py map:=<지도 폴더>
  로봇(luco_bringup 실행 중):
  ros2 launch luco3dnav_bringup localization.launch.py wheel_odom:=true use_description:=false use_static_tf:=false use_rviz:=false
  bag 재생: use_sensors:=false (bag 은 --clock 으로)

구성: mapping.launch.py (센서 + TF + FAST-LIO, PGO 없음)
      + localizer_node (지도 ICP, TF map -> lio_world)
      + global_localize (시작 때 스캔을 모아 지도 전체에서 위치 찾기 -> localizer 에 넘김)
      + Nav2 map_server (grid.yaml -> /map, 전역 경로 계획용) + RViz
기준 지도 정하기: ros2 run luco3dnav_mapping set_nav_map maps/pcd/<이름>
자동으로 못 찾으면 RViz "2D Pose Estimate" 로 대략 찍어 주면 됨.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def nodes(context):
    share = get_package_share_directory('luco3dnav_bringup')
    map_dir = LaunchConfiguration('map').perform(context)
    if not map_dir:
        from luco3dnav_mapping.set_nav_map import default_nav_dir
        map_dir = default_nav_dir()
    map_dir = os.path.abspath(os.path.expanduser(map_dir))
    if not os.path.isfile(os.path.join(map_dir, 'map.pcd')):
        raise RuntimeError(f'기준 지도 없음: {map_dir}/map.pcd  (ros2 run luco3dnav_mapping set_nav_map <지도 폴더>)')
    sim_time = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    out = [
        Node(package='localizer', executable='localizer_node', name='localizer_node', namespace='localizer',
             parameters=[{'config_path': os.path.join(share, 'config', 'localizer.yaml')}, sim_time],
             output='screen'),
        Node(package='luco3dnav_mapping', executable='global_localize', name='global_localize',
             parameters=[{'map_dir': map_dir}, sim_time], output='screen'),
    ]
    grid = os.path.join(map_dir, 'grid.yaml')
    try:
        get_package_share_directory('nav2_map_server')
        have_nav2 = True
    except Exception:  # noqa: BLE001  (PackageNotFoundError)
        have_nav2 = False
        print('[localization] nav2_map_server 없음 -> /map 발행 생략 '
              '(sudo apt install ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager)')
    if os.path.isfile(grid) and have_nav2:
        out += [
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 parameters=[{'yaml_filename': grid, 'frame_id': 'map'}, sim_time], output='screen',
                 condition=IfCondition(LaunchConfiguration('use_map_server'))),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
                 parameters=[{'autostart': True, 'node_names': ['map_server']}, sim_time],
                 condition=IfCondition(LaunchConfiguration('use_map_server'))),
        ]
    out.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                    arguments=['-d', os.path.join(share, 'rviz', 'localization.rviz')], parameters=[sim_time],
                    condition=IfCondition(LaunchConfiguration('use_rviz'))))
    return out


def generate_launch_description():
    share = get_package_share_directory('luco3dnav_bringup')
    passthrough = ['use_sensors', 'use_sim_time', 'wheel_odom', 'use_description', 'use_static_tf', 'imu_port']
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='', description='기준 지도 폴더 (기본: maps/nav)'),
        DeclareLaunchArgument('use_sensors', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value=PythonExpression(
            ["'false' if '", LaunchConfiguration('use_sensors'), "' == 'true' else 'true'"]),
                              description='bag 재생 시 true (use_sensors:=false 면 자동으로 true)'),
        DeclareLaunchArgument('wheel_odom', default_value='false'),
        DeclareLaunchArgument('use_description', default_value='true'),
        DeclareLaunchArgument('use_static_tf', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_map_server', default_value='true', description='grid.yaml -> /map'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'mapping.launch.py')),
            launch_arguments={**{k: LaunchConfiguration(k) for k in passthrough},
                              'use_pgo': 'false', 'use_rviz': 'false'}.items()),
        OpaqueFunction(function=nodes),
    ])
