from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            description='FCU URL for MAVROS.'
        ),
        DeclareLaunchArgument(
            'gcs_url',
            description='GCS URL for MAVROS.'
        ),
        DeclareLaunchArgument(
            'tgt_system',
            description='Target system ID for MAVROS.'
        ),
        DeclareLaunchArgument(
            'tgt_component',
            description='Target component ID for MAVROS.'
        ),
        DeclareLaunchArgument(
            'pluginlists_yaml',
            description='Plugin lists YAML file for MAVROS.'
        ),
        DeclareLaunchArgument(
            'config_yaml',
            description='Configuration YAML file for MAVROS.'
        ),
        DeclareLaunchArgument(
            'log_output', default_value='screen',
            description='Log output.'
        ),
        DeclareLaunchArgument(
            'fcu_protocol', default_value='v2.0',
            description='FCU Protocol for MAVROS.'
        ),
        DeclareLaunchArgument(
            'respawn_mavros', default_value='false',
            description='Whether to respawn MAVROS node.'
        ),
        DeclareLaunchArgument(
            'namespace', default_value='mavros',
            description='Namespace for the MAVROS node.'
        ),

        Node(
            package='mavros',
            executable='mavros_node',
            namespace=LaunchConfiguration('namespace'),
            output=LaunchConfiguration('log_output'),
            parameters=[
                {'fcu_url': LaunchConfiguration('fcu_url')},
                {'gcs_url': LaunchConfiguration('gcs_url')},
                {'tgt_system': LaunchConfiguration('tgt_system')},
                {'tgt_component': LaunchConfiguration('tgt_component')},
                {'fcu_protocol': LaunchConfiguration('fcu_protocol')},
                Command(['cat ', LaunchConfiguration('pluginlists_yaml')]),
                Command(['cat ', LaunchConfiguration('config_yaml')])
            ]
        ),
    ])
