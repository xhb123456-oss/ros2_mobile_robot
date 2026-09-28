"""Room simulation + saved-map localization + official Nav2 navigation."""
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def merge_parameters(base, overrides):
    """Merge mappings recursively; scalars and lists replace existing values."""
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge_parameters(base[key], value)
        else:
            base[key] = value
    return base


def start_navigation(context):
    project = Path(get_package_share_directory('mobile_robot_sim'))
    bringup = Path(get_package_share_directory('nav2_bringup'))
    with (bringup / 'params/nav2_params.yaml').open(encoding='utf-8') as stream:
        defaults = yaml.safe_load(stream)
    with (project / 'config/navigation_overrides.yaml').open(encoding='utf-8') as stream:
        overrides = yaml.safe_load(stream)
    params = merge_parameters(defaults, overrides)
    trial_path = LaunchConfiguration('nav_overrides').perform(context)
    if trial_path:
        with Path(trial_path).expanduser().open(encoding='utf-8') as stream:
            trial = yaml.safe_load(stream)
        if not isinstance(trial, dict):
            raise ValueError('nav_overrides must contain a YAML parameter mapping')
        params = merge_parameters(params, trial)
    # Keep installed Nav2 plugin lists and behavior-tree defaults compatible
    # with that installation, instead of maintaining a second full copy.
    with tempfile.NamedTemporaryFile(
            mode='w', prefix='mobile_robot_nav2_', suffix='.yaml',
            encoding='utf-8', delete=False) as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
        params_path = Path(stream.name)

    def cleanup(context):
        # Only remove the exact temporary file created by this launch.
        params_path.unlink(missing_ok=True)
        return []

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / 'launch/navigation_launch.py')),
        launch_arguments={
            'namespace': '', 'use_sim_time': 'true', 'autostart': 'true',
            # Humble evaluates PythonExpression(['not ', use_composition]);
            # this argument must be a Python boolean literal, not YAML 'false'.
            'use_composition': 'False', 'params_file': str(params_path),
        }.items(),
    )
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        GroupAction(actions=[navigation], scoped=True),
    ]


def generate_launch_description():
    project = FindPackageShare('mobile_robot_sim')
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([project, 'launch', 'localization.launch.py'])),
        launch_arguments={
            'map': LaunchConfiguration('map'), 'use_rviz': 'false',
            'amcl_overrides': LaunchConfiguration('amcl_overrides'),
        }.items(),
    )
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', PathJoinSubstitution([project, 'rviz', 'navigation.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to saved map YAML'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'amcl_overrides', default_value='',
            description='Optional AMCL parameter YAML forwarded to localization launch'),
        DeclareLaunchArgument(
            'nav_overrides', default_value='',
            description='Optional extra Nav2 parameter YAML, merged after project defaults'),
        GroupAction(actions=[localization], scoped=True),
        OpaqueFunction(function=start_navigation),
        rviz,
    ])
