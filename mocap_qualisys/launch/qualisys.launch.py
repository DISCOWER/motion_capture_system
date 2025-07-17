import os
import sys

import launch
import launch_ros.actions

from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    wara_ps_arg = launch.actions.DeclareLaunchArgument(
        "wara_ps",
        default_value="false",
        description="Set connection to WARA-PS",
    )

    auto_bag_arg = launch.actions.DeclareLaunchArgument(
        "auto_bag",
        default_value="false",
        description="Set to true to automatically start the autobag service",
    )

    waraps_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wara_ps"), "launch", "lab_integration.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("wara_ps")),
    )

    auto_bag = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("auto_bag"), "launch", "auto_bag_launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("auto_bag")),
        launch_arguments={
            'base_dir': '/home/discower/server/bags',
        }.items(),
    )

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='server_address',
            default_value='10.0.0.10'
        ),
        launch.actions.DeclareLaunchArgument(
            name='server_base_port',
            default_value='22222'
        ),
        launch.actions.DeclareLaunchArgument(
            name='frame_rate',
            default_value='0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='max_accel',
            default_value='10.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='publish_tf',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_frame_id',
            default_value='mocap'
        ),
        launch.actions.DeclareLaunchArgument(
            name='udp_port',
            default_value='0',
            description='UDP port can be set to -1 to request a TCP stream,     setting 0 requests a random port, any other positive value      requests that specific port'
        ),
        launch.actions.DeclareLaunchArgument(
            name='qtm_protocol_version',
            default_value='18'
        ),
        launch_ros.actions.Node(
            package='mocap_qualisys',
            executable='mocap_qualisys_node',
            name='qualisys',
            output='screen',
            parameters=[
                {
                    'server_address': launch.substitutions.LaunchConfiguration('server_address')
                },
                {
                    'server_base_port': launch.substitutions.LaunchConfiguration('server_base_port')
                },
                {
                    'frame_rate': launch.substitutions.LaunchConfiguration('frame_rate')
                },
                {
                    'max_accel': launch.substitutions.LaunchConfiguration('max_accel')
                },
                {
                    'publish_tf': launch.substitutions.LaunchConfiguration('publish_tf')
                },
                {
                    'fixed_frame_id': launch.substitutions.LaunchConfiguration('fixed_frame_id')
                },
                {
                    'udp_port': launch.substitutions.LaunchConfiguration('udp_port')
                },
                {
                    'qtm_protocol_version': launch.substitutions.LaunchConfiguration('qtm_protocol_version')
                }
            ]
        ),
        wara_ps_arg,
        waraps_bridge,
        auto_bag_arg,
        auto_bag,
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
