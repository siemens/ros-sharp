# © Siemens AG, 2024
# Author: Mehmet Emre Cakal (emre.cakal@siemens.com/m.emrecakal@gmail.com)

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
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from tempfile import NamedTemporaryFile

def generate_launch_description():
    # Declare launch arguments
    gui_arg = DeclareLaunchArgument('gui', default_value='true', description='Whether to show GUI')
    urdf_file_arg = DeclareLaunchArgument('urdf_file', default_value='robot_description.urdf', description='URDF file name')
    package_name_arg = DeclareLaunchArgument('package_name', default_value='urdf_export_test', description='Package name containing URDF file')
    rviz_config_file_arg = DeclareLaunchArgument('rviz_config_file', default_value='robot_visualization.rviz', description='RViz configuration file')

    return LaunchDescription([
        gui_arg,
        urdf_file_arg,
        package_name_arg,
        rviz_config_file_arg,
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context, *args, **kwargs):
    urdf_file = LaunchConfiguration('urdf_file').perform(context)
    package_name = LaunchConfiguration('package_name').perform(context)
    rviz_config_file = LaunchConfiguration('rviz_config_file').perform(context)

    # Construct the URDF file path
    urdf_file_path = os.path.join(
        get_package_share_directory(package_name),
        urdf_file
    )

    # Read the URDF file content
    with open(urdf_file_path, 'r') as infp:
        robot_description = infp.read()

    # Path to the RViz configuration file
    rviz_config_file_path = os.path.join(
        get_package_share_directory("file_server2"),
        'rviz',
        rviz_config_file
    )

    # Read the RViz config file, replace the placeholder with the actual path, and write to a temporary file
    with open(rviz_config_file_path, 'r') as infp:
        rviz_config_content = infp.read().replace('Description File:', 'Description File: ' + urdf_file_path)

    tmp_rviz_config_file = NamedTemporaryFile(delete=False, suffix='.rviz')
    with open(tmp_rviz_config_file.name, 'w') as outfp:
        outfp.write(rviz_config_content)

    # Define nodes
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file_path]
    )

    return [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ]
