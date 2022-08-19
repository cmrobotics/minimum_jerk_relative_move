# #!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
#from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    minimum_jerk_ros_dir = get_package_share_directory('minimum_jerk_ros')
    minimum_jerk_ros_params = os.path.join(
        minimum_jerk_ros_dir, 'params', 'minimum_jerk_params.yaml')
    namespace = LaunchConfiguration('namespace',			default='')
    params_file = LaunchConfiguration(
        'params_file',		default=minimum_jerk_ros_params)
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time")
    use_sim_time = LaunchConfiguration("use_sim_time")

    param_substitutions = {
        "namespace": namespace,
        "use_sim_time": use_sim_time,
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        convert_types=True,
        param_rewrites=param_substitutions,
    )

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')]

    minimum_jerk_node = Node(
        package="minimum_jerk_ros",
        executable="minimum_jerk_ros",
        name="minimum_jerk_ros",
        output='screen',
        remappings=remappings,
        parameters=[configured_params],
    )

    minimum_jerk_manager = Node(
        package='cmr_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_relative',
            output='screen',
            parameters=[
            {'use_sim_time': use_sim_time},
            {'node_names': ["minimum_jerk_ros"]},
            {'autostart': True}
        ]
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(minimum_jerk_node)
    ld.add_action(minimum_jerk_manager)

    return ld
