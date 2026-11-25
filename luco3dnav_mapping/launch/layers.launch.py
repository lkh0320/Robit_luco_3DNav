"""FAST-LIO2 + PGO 위에 맵 레이어 올리기: 3D voxel / 2D occupancy (octomap), 2.5D elevation (grid_map).

  # bag으로 확인 (개발 PC): SLAM + 레이어 + RViz 한 번에
  ros2 launch luco3dnav_mapping layers.launch.py use_sensors:=false
  ros2 bag play bags/luco_1125_1934 --clock --topics /imu/data /velodyne_points

  # 하나만 켜기
  ros2 launch luco3dnav_mapping layers.launch.py use_sensors:=false elevation:=false   # octomap만 (voxel + 2D)
  ros2 launch luco3dnav_mapping layers.launch.py use_sensors:=false octomap:=false     # elevation만

  # SLAM은 따로 띄워 둔 상태에서 레이어만
  ros2 launch luco3dnav_mapping layers.launch.py slam:=false use_sim_time:=true

  # 로봇 실시간: 계산은 전부 로봇 PC, 개발 PC는 RViz만 (두 PC 모두 luco_net.sh source)
  #   로봇 PC (luco_bringup 실행 중):
  ros2 launch luco3dnav_mapping layers.launch.py wheel_odom:=true use_description:=false use_static_tf:=false use_rviz:=false
  #   로봇 PC (bringup 없이 센서만):
  ros2 launch luco3dnav_mapping layers.launch.py use_rviz:=false
  #   개발 PC (보기만):
  ros2 launch luco3dnav_mapping layers.launch.py slam:=false octomap:=false elevation:=false

토픽: /octomap_full, /occupied_cells_vis_array (3D voxel), /projected_map (2D occupancy),
      /elevation_mapper/elevation_map (2.5D)
리셋: ros2 service call /octomap_server/reset std_srvs/srv/Empty
      ros2 service call /elevation_mapper/reset std_srvs/srv/Empty
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('luco3dnav_mapping')
    bringup_share = get_package_share_directory('luco3dnav_bringup')
    octomap_cfg = os.path.join(share, 'config', 'octomap.yaml')
    elevation_cfg = os.path.join(share, 'config', 'elevation.yaml')
    rviz_cfg = os.path.join(share, 'rviz', 'layers.rviz')

    use_sensors = LaunchConfiguration('use_sensors')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_time = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='true',
                              description='luco3dnav_bringup mapping.launch.py(FAST-LIO2 + PGO)도 같이 실행'),
        DeclareLaunchArgument('use_sensors', default_value='true',
                              description='false면 센서 드라이버 없이 (rosbag 재생 시)'),
        DeclareLaunchArgument('use_sim_time',
                              default_value=PythonExpression(["'false' if '", use_sensors, "' == 'true' else 'true'"])),
        DeclareLaunchArgument('use_description', default_value='true'),
        DeclareLaunchArgument('wheel_odom', default_value='false'),
        DeclareLaunchArgument('use_static_tf', default_value='true'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('octomap', default_value='true', description='3D voxel + 2D occupancy'),
        DeclareLaunchArgument('elevation', default_value='true', description='2.5D elevation'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        # scoped 그룹: include에 넘긴 use_rviz:=false 가 밖(이 launch의 RViz)으로 새지 않게
        GroupAction(scoped=True, condition=IfCondition(LaunchConfiguration('slam')), actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'mapping.launch.py')),
            launch_arguments={
                'use_sensors': use_sensors,
                'use_sim_time': use_sim_time,
                'use_description': LaunchConfiguration('use_description'),
                'wheel_odom': LaunchConfiguration('wheel_odom'),
                'use_static_tf': LaunchConfiguration('use_static_tf'),
                'imu_port': LaunchConfiguration('imu_port'),
                'use_rviz': 'false',
            }.items())]),

        Node(package='octomap_server', executable='octomap_server_node', name='octomap_server',
             parameters=[octomap_cfg, sim_time],
             remappings=[('cloud_in', '/fastlio2/body_cloud')],
             output='screen', condition=IfCondition(LaunchConfiguration('octomap'))),
        Node(package='luco3dnav_mapping', executable='elevation_mapper', name='elevation_mapper',
             parameters=[elevation_cfg, sim_time],
             output='screen', condition=IfCondition(LaunchConfiguration('elevation'))),
        Node(package='rviz2', executable='rviz2', name='rviz2_layers', arguments=['-d', rviz_cfg],
             parameters=[sim_time], condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
