"""개발 PC에서 로봇 PC의 실시간 SLAM을 RViz로만 보기 (계산은 로봇 PC에서).

  # 두 PC 모두 luco 터미널에서 먼저: source .../luco3dnav_bringup/scripts/luco_net.sh
  # 로봇 PC:  ros2 launch luco_bringup luco.launch.py odom_tf:=false
  #           ros2 launch luco3dnav_bringup mapping.launch.py use_description:=false use_static_tf:=false use_rviz:=false
  # 개발 PC:  ros2 launch luco3dnav_bringup view.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory('luco3dnav_bringup'), 'rviz', 'mapping.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        LogInfo(msg=['[view] ROS_DOMAIN_ID=', EnvironmentVariable('ROS_DOMAIN_ID', default_value='0'),
                     ' RMW=', EnvironmentVariable('RMW_IMPLEMENTATION', default_value='(기본)'),
                     '  -> 로봇과 다르면 luco_net.sh 를 source 했는지 확인']),
        Node(package='rviz2', executable='rviz2', name='rviz2_remote', arguments=['-d', rviz_cfg],
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
