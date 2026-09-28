"""Launch the room simulation, official SLAM Toolbox, and mapping RViz."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    project = FindPackageShare('mobile_robot_sim')
    slam = FindPackageShare('slam_toolbox')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([project, 'launch', 'simulation.launch.py'])),
        launch_arguments={
            'world_file': 'mapping_room.world',
            'use_rviz': 'false',
        }.items(),
    )
    # Preserve the installed official solver/scan-matcher defaults; override
    # only this robot's frames, sensor limits, and small-room sampling settings.
    mapper = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            PathJoinSubstitution([slam, 'config', 'mapper_params_online_async.yaml']),
            PathJoinSubstitution([project, 'config', 'slam_mapping.yaml']),
        ],
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
        DeclareLaunchArgument('use_rviz', default_value='true'),
        # The child disables its own RViz. Scope that argument so it cannot
        # overwrite this launch file's use_rviz setting for the mapping view.
        GroupAction(actions=[simulation], scoped=True),
        mapper,
        rviz,
    ])
