import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='server_address',
            default_value='qtm-pc'
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
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
