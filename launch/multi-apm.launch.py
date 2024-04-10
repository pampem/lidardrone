from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # MAVROSパッケージのディレクトリパスを取得
    mavros_pkg_dir = get_package_share_directory('mavros')

    # 設定ファイルのフルパスを指定
    apm_config_path = os.path.join(mavros_pkg_dir, 'launch', 'apm_config.yaml')
    apm_pluginlists_path = os.path.join(mavros_pkg_dir, 'launch', 'apm_pluginlists.yaml')

    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            namespace='/drone1',
            output='screen',
            respawn=True,
            parameters=[apm_config_path, apm_pluginlists_path,
                        {'fcu_url': 'udp://127.0.0.1:14551@14555'},
                        {'gcs_url': ''},
                        {'target_system_id': 1},
                        {'target_component_id': 1},
                        {'fcu_protocol': 'v2.0'}]
        ),
        # 他のドローンのノード定義も同様にここに追加
    ])
