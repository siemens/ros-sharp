# © Siemens AG, 2024
# Author: Mehmet Emre Cakal (emre.cakal@siemens.com)

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# <http://www.apache.org/licenses/LICENSE-2.0>.
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Define launch arguments
    port_arg = DeclareLaunchArgument('port', 
                                    default_value='9090', 
                                    description='Port number for ROS communication (default: 9090)')
    
    fragment_timeout_arg = DeclareLaunchArgument('fragment_timeout',
                                    default_value='600', 
                                    description='Timeout for fragment reassembly in seconds (default: 600)')
    
    unregister_timeout_arg = DeclareLaunchArgument('unregister_timeout',
                                    default_value='10.0',
                                    description='Timeout for unregistering in seconds (default: 10.0)')
    
    max_message_size_arg = DeclareLaunchArgument('max_message_size',
                                    default_value='100000000',
                                    description='Maximum message size in bytes (default: 10000000)')

    allow_save_arg = DeclareLaunchArgument('allow_save',
                                    default_value='false',
                                    description='Enable saving files (default: false)')

    allow_overwrite_arg = DeclareLaunchArgument('allow_overwrite',
                                    default_value='false',
                                    description='Allow overwrite of existing files (default: false)')

    allow_file_url_arg = DeclareLaunchArgument('allow_file_url',
                                    default_value='false',
                                    description='Allow read access to "file://" URLs. (default: false)')

    allow_file_url_root_arg = DeclareLaunchArgument('allow_file_url_root',
                                    default_value='/opt/ros/',
                                    description='"file://" URLs are restricted to this root. (default: /opt/ros/)')

    log_level_arg = DeclareLaunchArgument('log_level',
                                    default_value='info',
                                    description='Log level for rosbridge (default: info)')

    return LaunchDescription([
        port_arg,
        fragment_timeout_arg,
        unregister_timeout_arg,
        max_message_size_arg,
        allow_save_arg,
        allow_overwrite_arg,
        allow_file_url_arg,
        allow_file_url_root_arg,
        log_level_arg,


        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            ros_arguments=['--log-level', LaunchConfiguration('log_level')],
            parameters=[{
                'port': LaunchConfiguration('port'),
                'fragment_timeout': LaunchConfiguration('fragment_timeout'),
                'unregister_timeout': LaunchConfiguration('unregister_timeout'),
                'max_message_size': LaunchConfiguration('max_message_size'),
            }]
        ),

        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
        ),
        
        Node(
            package='file_server2',
            executable='file_server2_node',
            output='screen',
            parameters=[
                {'allow_save': LaunchConfiguration('allow_save')},
                {'allow_overwrite': LaunchConfiguration('allow_overwrite')},
                {'allow_file_url': LaunchConfiguration('allow_file_url')},
                {'allow_file_url_root': LaunchConfiguration('allow_file_url_root')}
            ]
        )
    ])

