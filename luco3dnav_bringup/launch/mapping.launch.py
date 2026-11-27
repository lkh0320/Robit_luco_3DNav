"""Mapping: sensors + (static TF) + FAST-LIO2 + PGO + RViz.

  ros2 launch luco3dnav_bringup mapping.launch.py                          # 로봇 모델(luco_description) 포함
  ros2 launch luco3dnav_bringup mapping.launch.py use_description:=false   # 모델 없이 정적 TF만

  로봇(luco_bringup 실행 중): mapping.launch.py wheel_odom:=true use_description:=false use_static_tf:=false
  지도 레이어(2D/voxel/elevation)까지: luco3dnav_mapping layers.launch.py (이 launch를 포함, 인자 같음)
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
    wheel_odom = LaunchConfiguration('wheel_odom')
    use_rviz = LaunchConfiguration('use_rviz')
    use_pgo = LaunchConfiguration('use_pgo')
    imu_port = LaunchConfiguration('imu_port')
    # bag 데이터 시각과 노드 시각을 맞추기 위해 /clock 사용 (bool 파라미터로 전달)
    sim_time = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}

    # luco_description URDF와 같은 값 (실측 2026-11-25, base_link 기준)
    #   lio.yaml t_bi는 base_footprint 기준이라 아래 base_link 기준 값보다 z가 0.0613 큼
    # use_description:=true(기본)면 robot_state_publisher가 같은 TF를 주므로 정적 TF는 자동으로 꺼짐
    static_tf_on = IfCondition(PythonExpression(
        ["'", use_static_tf, "' == 'true' and '", use_description, "' != 'true'"]))
    static_tf = [
        # base_footprint(바닥) -> base_link: luco_description base_joint (z = 0.1143 - 0.053)
        Node(package='tf2_ros', executable='static_transform_publisher', name='footprint_to_base',
             arguments=['--z', '0.0613', '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
             parameters=[sim_time], condition=static_tf_on),
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_velodyne',
             arguments=['--x', '0.12545', '--y', '0.0', '--z', '0.3064',
                        '--frame-id', 'base_link', '--child-frame-id', 'velodyne'],
             parameters=[sim_time], condition=static_tf_on),
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_imu',
             arguments=['--x', '0.101', '--y', '0.00225', '--z', '0.2548', '--yaw', '-1.5707963',
                        '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
             parameters=[sim_time], condition=static_tf_on),
    ]

    # 로봇 모델: luco_description URDF -> robot_description
    #   TF: base_link -> velodyne/imu_link/바퀴/카메라 마스트(head_camera_link)/매니퓰레이터(wrist_camera_link)
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution(
            [FindPackageShare('luco_description'), 'urdf', 'luco_description.urdf.xacro'])]),
        value_type=str)
    description = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', parameters=[{'robot_description': robot_description}, sim_time],
             condition=IfCondition(use_description)),
        # 움직이는 조인트 TF용: 바퀴·카메라 팬·팔·그리퍼를 전부 0으로 고정
        #   (0 = CAD 자세 = 팔 접힘, 카메라 정면 뒤쪽). 실제 팔 각도가 필요하면 팔 드라이버의 /joint_states를 쓸 것
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', parameters=[sim_time], condition=IfCondition(use_description)),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sensors', default_value='true',
                              description='false면 센서 드라이버 없이 (rosbag 재생 시)'),
        DeclareLaunchArgument('use_sim_time',
                              default_value=PythonExpression(["'false' if '", use_sensors, "' == 'true' else 'true'"]),
                              description='bag 재생 시 true (기본: use_sensors:=false면 true)'),
        DeclareLaunchArgument('wheel_odom', default_value='false',
                              description='true: luco_bringup 휠 오도메트리(odom->base_footprint)와 연동, '
                                          'FAST-LIO는 lio_world->odom 발행'),
        DeclareLaunchArgument('use_description', default_value='true',
                              description='luco_description 로봇 모델 + TF 사용'),
        DeclareLaunchArgument('use_static_tf', default_value='true',
                              description='use_description:=false일 때 정적 TF 발행'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_pgo', default_value='true',
                              description='false: 지도 만들기 없이 (localization.launch.py 가 씀)'),
        DeclareLaunchArgument('pgo_cloud', default_value='/fastlio2/body_cloud',
                              description='PGO 키프레임 점군 입력 (layers.launch.py 가 동적 필터 결과로 바꿈)'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sensors_share, 'launch', 'sensors.launch.py')),
            launch_arguments={'imu_port': imu_port}.items(),
            condition=IfCondition(use_sensors)),

        *static_tf,
        *description,

        Node(package='fastlio2', executable='lio_node', name='lio_node', namespace='fastlio2',
             parameters=[{'config_path': lio_cfg, 'odom_frame': PythonExpression(
                 ["'odom' if '", wheel_odom, "' == 'true' else '__yaml__'"])}, sim_time], output='screen'),
        Node(package='pgo', executable='pgo_node', name='pgo_node', namespace='pgo',
             parameters=[{'config_path': pgo_cfg}, sim_time], output='screen', condition=IfCondition(use_pgo),
             remappings=[('/fastlio2/body_cloud', LaunchConfiguration('pgo_cloud'))]),
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_cfg],
             parameters=[sim_time],
             condition=IfCondition(use_rviz)),
    ])
