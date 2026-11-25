"""개발 PC에서 로봇 PC의 실시간 SLAM을 RViz로만 보기 (계산은 로봇 PC에서).

  # 두 PC 모두 luco 터미널에서 먼저: source .../luco3dnav_bringup/scripts/luco_net.sh  (개발 PC alias: luco_net)
  # 로봇 PC:  ros2 launch luco_bringup luco.launch.py
  #           ros2 launch luco3dnav_mapping layers.launch.py wheel_odom:=true use_description:=false use_static_tf:=false use_rviz:=false
  #           (레이어 없이 SLAM만이면 luco3dnav_bringup mapping.launch.py, 인자 같음)
  # 개발 PC:  ros2 launch luco3dnav_bringup view.launch.py                # SLAM + 레이어(2D/voxel/elevation) 화면
  #           ros2 launch luco3dnav_bringup view.launch.py layers:=false  # SLAM 화면만 (점군/경로/루프)

  bag을 다른 PC에서 재생하는 걸 원격으로 볼 때는 use_sim_time:=true
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _rviz(context):
    use_layers = LaunchConfiguration('layers').perform(context).lower() == 'true'
    sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    cfg = os.path.join(get_package_share_directory('luco3dnav_bringup'), 'rviz', 'mapping.rviz')
    msg = 'SLAM 화면 (mapping.rviz)'
    if use_layers:
        # luco3dnav_mapping 이 bringup 에 의존하므로 반대 의존은 두지 않고 실행 시점에 찾음
        try:
            cfg = os.path.join(get_package_share_directory('luco3dnav_mapping'), 'rviz', 'layers.rviz')
            msg = 'SLAM + 레이어 화면 (layers.rviz). 로봇이 mapping.launch.py만 돌리면 레이어 칸은 비어 있음'
        except PackageNotFoundError:
            msg = 'luco3dnav_mapping 이 없어 SLAM 화면 (mapping.rviz)으로 대신'
    return [
        LogInfo(msg=f'[view] {msg}'),
        Node(package='rviz2', executable='rviz2', name='rviz2_remote', arguments=['-d', cfg],
             parameters=[{'use_sim_time': sim_time}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('layers', default_value='true',
                              description='true: SLAM + 2D/voxel/elevation 레이어 화면, false: SLAM 화면만'),
        LogInfo(msg=['[view] ROS_DOMAIN_ID=', EnvironmentVariable('ROS_DOMAIN_ID', default_value='0'),
                     ' RMW=', EnvironmentVariable('RMW_IMPLEMENTATION', default_value='(기본)'),
                     '  -> 로봇과 다르면 luco_net.sh 를 source 했는지 확인']),
        OpaqueFunction(function=_rviz),
    ])
