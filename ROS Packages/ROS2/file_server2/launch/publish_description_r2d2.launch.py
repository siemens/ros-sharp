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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define launch arguments
    port_arg = DeclareLaunchArgument('port', default_value='9090', description='Port number for ROS communication')
    urdf_file_arg = DeclareLaunchArgument('urdf_file', default_value='custom_r2d2.urdf', description='URDF file name')

    # Define paths
    pkg_file_server2 = get_package_share_directory('file_server2')
    rosbridge_server_launch_file = PathJoinSubstitution(
        [pkg_file_server2, 'launch', 'ros_sharp_communication.launch.py'])
    
    urdf_launch_path = get_package_share_directory('urdf_launch')
    urdf_launch_launch_file = PathJoinSubstitution(
        [urdf_launch_path, 'launch', 'description.launch.py'])
    
    custom_urdf_file_path = PathJoinSubstitution([
        pkg_file_server2,
        "urdf",
        LaunchConfiguration('urdf_file')
    ])
    
    # Include ROS Sharp Communication launch file
    ros_sharp_communication_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbridge_server_launch_file]),
                launch_arguments=[('port', LaunchConfiguration('port'))]
        )
    
    r2d2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([urdf_launch_launch_file]),
        launch_arguments={
            'urdf_package': 'urdf_tutorial',
            'urdf_package_path': custom_urdf_file_path}.items()
    ) 

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    
    return LaunchDescription([
        port_arg,
        urdf_file_arg,
        ros_sharp_communication_launch,
        r2d2_launch,
        joint_state_publisher_node
    ])

