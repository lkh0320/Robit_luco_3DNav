"""EBIMU(/imu/data) + Velodyne VLP-16(/velodyne_points)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('luco3dnav_sensors')
    ebimu_cfg = os.path.join(share, 'config', 'ebimu.yaml')
    velo_cfg = os.path.join(share, 'config', 'velodyne.yaml')
    velo_calib = os.path.join(
        get_package_share_directory('velodyne_pointcloud'), 'params', 'VLP16db.yaml')

    use_imu = LaunchConfiguration('use_imu')
    use_lidar = LaunchConfiguration('use_lidar')
    imu_port = LaunchConfiguration('imu_port')

    return LaunchDescription([
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB0'),

        Node(package='luco3dnav_sensors', executable='ebimu_imu_node', name='ebimu_imu_node',
             parameters=[ebimu_cfg, {'port': imu_port}], output='screen',
             condition=IfCondition(use_imu)),

        Node(package='velodyne_driver', executable='velodyne_driver_node',
             name='velodyne_driver_node', parameters=[velo_cfg], output='screen',
             condition=IfCondition(use_lidar)),
        Node(package='velodyne_pointcloud', executable='velodyne_transform_node',
             name='velodyne_transform_node', parameters=[velo_cfg, {'calibration': velo_calib}],
             output='screen', condition=IfCondition(use_lidar)),
    ])
