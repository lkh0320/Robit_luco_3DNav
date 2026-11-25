"""Mapping: sensors + (static TF) + FAST-LIO2 + PGO + RViz.

  ros2 launch luco3dnav_bringup mapping.launch.py                          # 로봇 모델(luco2_description) 포함
  ros2 launch luco3dnav_bringup mapping.launch.py use_description:=false   # 모델 없이 정적 TF만

  bag 재생: use_sensors:=false 면 use_sim_time이 자동으로 true -> bag은 반드시 --clock 으로 재생
  ros2 launch luco3dnav_bringup mapping.launch.py use_sensors:=false
  ros2 bag play <bag> --clock --topics /imu/data /velodyne_points
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory('luco3dnav_bringup')
    sensors_share = get_package_share_directory('luco3dnav_sensors')
    lio_cfg = os.path.join(share, 'config', 'lio.yaml')
    pgo_cfg = os.path.join(share, 'config', 'pgo.yaml')
    rviz_cfg = os.path.join(share, 'rviz', 'mapping.rviz')

    use_sensors = LaunchConfiguration('use_sensors')
    use_static_tf = LaunchConfiguration('use_static_tf')
    use_description = LaunchConfiguration('use_description')
    use_rviz = LaunchConfiguration('use_rviz')
    imu_port = LaunchConfiguration('imu_port')
    # bag 데이터 시각과 노드 시각을 맞추기 위해 /clock 사용 (bool 파라미터로 전달)
    sim_time = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}

    # luco2_description URDF와 같은 값 (실측 2026-11-25, base_link 기준)
    #   imu_link = lio.yaml t_bi, velodyne = t_bi + t_il
    # use_description:=true(기본)면 robot_state_publisher가 같은 TF를 주므로 정적 TF는 자동으로 꺼짐
    static_tf_on = IfCondition(PythonExpression(
        ["'", use_static_tf, "' == 'true' and '", use_description, "' != 'true'"]))
    static_tf = [
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_velodyne',
             arguments=['--x', '0.12545', '--y', '0.0', '--z', '0.3064',
                        '--frame-id', 'base_link', '--child-frame-id', 'velodyne'],
             parameters=[sim_time], condition=static_tf_on),
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_imu',
             arguments=['--x', '0.101', '--y', '0.00225', '--z', '0.2548', '--yaw', '-1.5707963',
                        '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
             parameters=[sim_time], condition=static_tf_on),
    ]

    # 로봇 모델: luco2_description URDF -> robot_description, TF(base_link->velodyne/imu_link/바퀴)
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution(
            [FindPackageShare('luco2_description'), 'urdf', 'luco2.xacro'])]),
        value_type=str)
    description = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', parameters=[{'robot_description': robot_description}, sim_time],
             condition=IfCondition(use_description)),
        # 바퀴 continuous 조인트 TF용 (휠 엔코더 없으니 0으로 고정)
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', parameters=[sim_time], condition=IfCondition(use_description)),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sensors', default_value='true',
                              description='false면 센서 드라이버 없이 (rosbag 재생 시)'),
        DeclareLaunchArgument('use_sim_time',
                              default_value=PythonExpression(["'false' if '", use_sensors, "' == 'true' else 'true'"]),
                              description='bag 재생 시 true (기본: use_sensors:=false면 true)'),
        DeclareLaunchArgument('use_description', default_value='true',
                              description='luco2_description 로봇 모델 + TF 사용'),
        DeclareLaunchArgument('use_static_tf', default_value='true',
                              description='use_description:=false일 때 정적 TF 발행'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sensors_share, 'launch', 'sensors.launch.py')),
            launch_arguments={'imu_port': imu_port}.items(),
            condition=IfCondition(use_sensors)),

        *static_tf,
        *description,

        Node(package='fastlio2', executable='lio_node', name='lio_node', namespace='fastlio2',
             parameters=[{'config_path': lio_cfg}, sim_time], output='screen'),
        Node(package='pgo', executable='pgo_node', name='pgo_node', namespace='pgo',
             parameters=[{'config_path': pgo_cfg}, sim_time], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_cfg],
             parameters=[sim_time],
             condition=IfCondition(use_rviz)),
    ])
