"""独立终点对正实验入口；默认仍使用原始巡线控制链。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build_nodes(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    enabled = value('enable_finish_heading').lower()
    use_yolo = value('use_yolo').lower()
    if enabled not in ('true', 'false') or use_yolo not in ('true', 'false'):
        raise ValueError('enable_finish_heading and use_yolo must be true or false')

    share = get_package_share_directory('line_follower')
    params = value('params_file')
    image_topic = value('image_topic')
    if enabled == 'false':
        launch_name = ('line_follower_with_detection.launch.py' if use_yolo == 'true'
                       else 'line_follower.launch.py')
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', launch_name)),
            launch_arguments={'params_file': params, 'image_topic': image_topic,
                              'cmd_vel_topic': '/cmd_vel'}.items())]

    calibration = value('calibration_file')
    if not calibration:
        raise ValueError('Explicit surveyed calibration_file is required')

    actions = []
    if use_yolo == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'line_follower_with_detection.launch.py')),
            launch_arguments={'params_file': params, 'image_topic': image_topic,
                              'cmd_vel_topic': '/finish_heading/request'}.items()))
    else:
        # The original C++ node is redirected to the gate's private request topic.
        # The gate becomes the only publisher of the chassis-facing /cmd_vel.
        actions.append(Node(
            package='line_follower', executable='line_follower_node',
            name='line_follower_node', output='screen',
            parameters=[params, {'image_topic': image_topic,
                                 'cmd_vel_topic': '/finish_heading/request'}]))

    actions.append(Node(
        package='line_follower', executable='finish_heading_node.py',
        name='finish_heading_node', output='screen', parameters=[{
            'image_topic': image_topic,
            'calibration_file': calibration,
            'request_topic': '/finish_heading/request',
            'output_topic': '/cmd_vel',
        }]))
    return actions


def generate_launch_description():
    share = get_package_share_directory('line_follower')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            share, 'config', 'line_follower_params.yaml')),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('enable_finish_heading', default_value='false'),
        DeclareLaunchArgument('use_yolo', default_value='false'),
        OpaqueFunction(function=build_nodes),
    ])
