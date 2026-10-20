"""
GMapping SLAM for the TurtleBot4 / ICTE4001Robot (ROS 2 Humble).

    ros2 launch tb4_gmapping_nav gmapping.launch.py
    ros2 launch tb4_gmapping_nav gmapping.launch.py rviz:=true
    ros2 launch tb4_gmapping_nav gmapping.launch.py use_sim_time:=false    # real robot

Subscribes : /scan (sensor_msgs/LaserScan), TF odom->base_link (odometry), TF base_link->laser
Publishes  : /map (nav_msgs/OccupancyGrid, latched), /map_metadata, /entropy (std_msgs/Float64),
             /slam_gmapping/particles (geometry_msgs/PoseArray), TF map->odom
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('tb4_gmapping_nav')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    scan_topic = LaunchConfiguration('scan_topic')
    rviz = LaunchConfiguration('rviz')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='true in Gazebo, false on the physical TurtleBot4'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'gmapping.yaml'),
            description='GMapping parameter file'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='2D LaserScan topic of the robot'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start RViz with the GMapping view (map, scan, particles)'),
        DeclareLaunchArgument('log_level', default_value='info'),

        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            output='screen',
            # params_file first, then use_sim_time so the launch argument wins
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('scan', scan_topic)],
            arguments=['--ros-args', '--log-level', log_level]),

        Node(
            condition=IfCondition(rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2_gmapping',
            output='log',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'gmapping.rviz')],
            parameters=[{'use_sim_time': use_sim_time}]),
    ])
