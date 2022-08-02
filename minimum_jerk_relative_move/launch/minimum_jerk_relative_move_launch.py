import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node
from launch.actions import LogInfo, DeclareLaunchArgument
from launch.conditions import IfCondition
from cmr_launch_utils.substitutions import IfElseSubstitution


def generate_launch_description():
  namespace = LaunchConfiguration('namespace', default='')
  detection_params = os.path.join(get_package_share_directory("shelf_detector"), "shelf_detector_params.yaml")
  params_file = LaunchConfiguration('params_file', default=detection_params)
  use_sim_time = LaunchConfiguration('use_sim_time', default=False)
  stop_distance_trolley_detector = LaunchConfiguration('stop_distance_trolley_detector', default='3.0')
  autostart_trolley_detector = LaunchConfiguration('autostart_trolley_detector', default='False')
  is_gazebo_simulation = LaunchConfiguration('is_gazebo_simulation')

  remappings = [
    ('/tf', 'tf'),
    ('/tf_static', 'tf_static'),
    ('scan_in', IfElseSubstitution(
                    condition=IfCondition(is_gazebo_simulation),
                    if_true=TextSubstitution(text="scan"),
                    if_false=TextSubstitution(text="filtered_scan")
                ))
  ]

  param_substitutions = {
    'namespace': namespace, 
    'use_sim_time': use_sim_time,
    'stop_distance': stop_distance_trolley_detector,
    'activate_on_start': autostart_trolley_detector, 
  }

  configure_params = RewrittenYaml(
    source_file    = params_file,
    root_key       = namespace,
    param_rewrites = param_substitutions,
    convert_types  = True
  )

  return LaunchDescription([
    # -r scan_in:=/filtered_scan
    Node(
      package    = 'pointcloud_to_laserscan',
      executable = 'laserscan_to_pointcloud_node',
      name       = 'laserscan_to_pointcloud_node',
      output     = 'screen',
      remappings = remappings,
      parameters = [configure_params],
    ),

    Node(
      package    = 'laser_filters',
      executable = 'intensity_filter',
      name       = 'intensity_filter',
      output     = 'screen',
      remappings = remappings,
      parameters = [configure_params],
    ),


    Node(
      package    = 'shelf_detector',
      executable = 'shelf_detector',
      name       = 'shelf_detector',
      output     = 'screen',
      remappings = remappings,
      parameters = [configure_params],
      arguments  = {'shelf_pose_frame_id': LaunchConfiguration('laser')}),


    Node(
      package    = 'shelf_detector',
      executable = 'precision_trolley_detector',
      name       = 'precision_trolley_detector',
      output     = 'screen',
      remappings = remappings,
      parameters = [configure_params])
  ])
