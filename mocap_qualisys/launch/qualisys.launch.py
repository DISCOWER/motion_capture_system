import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare arguments with their default values
    declare_server_address_arg = DeclareLaunchArgument(
        'server_address', default_value='qtm-pc',
        description='Server address argument')

    declare_server_base_port_arg = DeclareLaunchArgument(
        'server_base_port', default_value='22222',
        description='Server base port argument')

    declare_frame_rate_arg = DeclareLaunchArgument(
        'frame_rate', default_value='0',
        description='Frame rate argument')

    declare_max_accel_arg = DeclareLaunchArgument(
        'max_accel', default_value='10.0',
        description='Max acceleration argument')

    declare_publish_tf_arg = DeclareLaunchArgument(
        'publish_tf', default_value='true',
        description='Publish tf argument')

    declare_fixed_frame_id_arg = DeclareLaunchArgument(
        'fixed_frame_id', default_value='mocap',
        description='Fixed frame ID argument')

    declare_udp_port_arg = DeclareLaunchArgument(
        'udp_port', default_value='0',
        description='UDP port argument. -1 for TCP stream, 0 for random port.')

    declare_qtm_protocol_version_arg = DeclareLaunchArgument(
        'qtm_protocol_version', default_value='18',
        description='QTM protocol version argument')

    # Node definition
    mocap_qualisys_node = Node(
        package='mocap_qualisys',
        executable='mocap_qualisys_node',
        name='qualisys',
        output='screen',
        parameters=[
            {'server_address': LaunchConfiguration('server_address')},
            {'server_base_port': LaunchConfiguration('server_base_port')},
            {'frame_rate': LaunchConfiguration('frame_rate')},
            {'max_accel': LaunchConfiguration('max_accel')},
            {'publish_tf': LaunchConfiguration('publish_tf')},
            {'fixed_frame_id': LaunchConfiguration('fixed_frame_id')},
            {'udp_port': LaunchConfiguration('udp_port')},
            {'qtm_protocol_version': LaunchConfiguration('qtm_protocol_version')},
            {'model_list': []}  # This is an empty list for now. Modify as necessary.
        ],
        # Uncomment the remap line and modify as necessary
        # remappings=[('qualisys/F450/odom', '/f450/odom')]
    )

    return LaunchDescription([
        declare_server_address_arg,
        declare_server_base_port_arg,
        declare_frame_rate_arg,
        declare_max_accel_arg,
        declare_publish_tf_arg,
        declare_fixed_frame_id_arg,
        declare_udp_port_arg,
        declare_qtm_protocol_version_arg,
        mocap_qualisys_node
    ])
