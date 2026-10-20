"""
GMapping SLAM + Nav2 autonomous navigation on the map being built.

    # 1) start the simulator (see README), then:
    ros2 launch tb4_gmapping_nav slam_nav.launch.py
    ros2 launch tb4_gmapping_nav slam_nav.launch.py planner:=iar_astar   # Practical 7 A* planner
    ros2 launch tb4_gmapping_nav slam_nav.launch.py record:=true run_name:=office_run1

Composition:
    slam_gmapping  --/map, TF map->odom-->  Nav2 (planner_server, controller_server,
                                                 bt_navigator, behavior_server, ...)
    Nav2 replaces AMCL + map_server: localisation (map->odom) comes from GMapping.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('tb4_gmapping_nav')
    nav2_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params = LaunchConfiguration('slam_params_file')
    planner = LaunchConfiguration('planner')
    rviz = LaunchConfiguration('rviz')
    record = LaunchConfiguration('record')
    run_name = LaunchConfiguration('run_name')
    gt_topic = LaunchConfiguration('gt_topic')
    gt_type = LaunchConfiguration('gt_type')
    gt_child_frame = LaunchConfiguration('gt_child_frame')
    nav2_delay = LaunchConfiguration('nav2_delay')

    # planner:=navfn -> config/nav2_params.yaml, planner:=iar_astar -> config/nav2_params_iar_astar.yaml
    nav2_params = PathJoinSubstitution([
        pkg_dir, 'config',
        PythonExpression(["'nav2_params_iar_astar.yaml' if '", planner,
                          "' == 'iar_astar' else 'nav2_params.yaml'"])])

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_dir, 'config', 'gmapping.yaml')),
        DeclareLaunchArgument(
            'planner', default_value='navfn',
            description="'navfn' (Nav2 default) or 'iar_astar' (Practical 7 planner)"),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'record', default_value='false',
            description='Run slam_eval_recorder to log poses/entropy to CSV'),
        DeclareLaunchArgument('run_name', default_value='run'),
        DeclareLaunchArgument(
            'gt_topic', default_value='',
            description='Optional ground-truth pose topic (simulation only)'),
        DeclareLaunchArgument('gt_type', default_value='odometry',
                              description="'odometry' (nav_msgs/Odometry) or 'tf' (tf2_msgs/TFMessage)"),
        DeclareLaunchArgument('gt_child_frame', default_value='turtlebot4',
                              description="entity name to pick from a TFMessage ground-truth topic"),
        DeclareLaunchArgument(
            'nav2_delay', default_value='5.0',
            description='[s] wait for GMapping to publish the first map before starting Nav2'),
    ]

    gmapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'gmapping.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': slam_params,
            'rviz': 'false',
        }.items())

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
            'use_composition': 'False',
        }.items())

    rviz_node = Node(
        condition=IfCondition(rviz),
        package='rviz2', executable='rviz2', name='rviz2_gmapping', output='log',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'gmapping.rviz')],
        parameters=[{'use_sim_time': use_sim_time}])

    recorder = Node(
        condition=IfCondition(record),
        package='tb4_gmapping_nav', executable='slam_eval_recorder.py',
        name='slam_eval_recorder', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'run_name': run_name,
            'gt_topic': gt_topic,
            'gt_type': gt_type,
            'gt_child_frame': gt_child_frame,
        }])

    return LaunchDescription(declare + [
        gmapping,
        rviz_node,
        recorder,
        # Nav2's global costmap needs the map->odom TF and a first /map, so start it a bit later
        TimerAction(period=nav2_delay, actions=[nav2]),
    ])
