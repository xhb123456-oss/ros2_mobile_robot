"""Restart the room and localize on a saved map, without running SLAM."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def start_amcl(context):
    project = FindPackageShare('mobile_robot_sim')
    params = [PathJoinSubstitution([project, 'config', 'localization.yaml'])]
    trial_path = LaunchConfiguration('amcl_overrides').perform(context)
    if trial_path:
        trial = Path(trial_path).expanduser().resolve()
        if not trial.is_file():
            raise FileNotFoundError(f'AMCL override file does not exist: {trial}')
        params.append(str(trial))
    return [Node(
        package='nav2_amcl', executable='amcl', name='amcl',
        parameters=params, output='screen',
    )]


def generate_launch_description():
    project = FindPackageShare('mobile_robot_sim')
    params = PathJoinSubstitution([project, 'config', 'localization.yaml'])
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([project, 'launch', 'simulation.launch.py'])),
        launch_arguments={
            'world_file': 'mapping_room.world', 'use_rviz': 'false',
        }.items(),
    )
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        parameters=[params, {
            'yaml_filename': ParameterValue(LaunchConfiguration('map'), value_type=str),
        }],
        output='screen',
    )
    amcl = OpaqueFunction(function=start_amcl)
    manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
        output='screen',
    )
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', PathJoinSubstitution([project, 'rviz', 'mapping.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to saved map YAML'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'amcl_overrides', default_value='',
            description='Optional AMCL parameter YAML, applied after localization defaults'),
        GroupAction(actions=[simulation], scoped=True),
        map_server, amcl, manager, rviz,
    ])
