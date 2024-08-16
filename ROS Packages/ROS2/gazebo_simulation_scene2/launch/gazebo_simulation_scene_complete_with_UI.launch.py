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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Declare launch arguments
    urdf_file_arg = DeclareLaunchArgument('urdf_file', default_value='custom_r2d2.urdf', description='URDF file name')
    bridge_config_file_arg = DeclareLaunchArgument('bridge_config_file', default_value='gazebo_bridge_effort.yaml', description='Bridge config file name')
    package_name_arg = DeclareLaunchArgument('package_name', default_value='gazebo_simulation_scene2', description='Package name containing URDF file')
    gazebo_world_arg = DeclareLaunchArgument('gazebo_world', default_value='gazebo_simulation_scene2_world.sdf', description='Gazebo world file name')


    return LaunchDescription([
        urdf_file_arg,
        bridge_config_file_arg,
        package_name_arg,
        gazebo_world_arg,
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context, *args, **kwargs):
    urdf_file = LaunchConfiguration('urdf_file').perform(context)
    bridge_config_file = LaunchConfiguration('bridge_config_file').perform(context)
    gazebo_world_file = LaunchConfiguration('gazebo_world').perform(context)
    package_name = LaunchConfiguration('package_name').perform(context)

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_file_server2 = get_package_share_directory('file_server2')

    urdf_file_path = os.path.join(
        get_package_share_directory("file_server2"),
        "urdf",
        urdf_file
    )

    bridge_config_file_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        bridge_config_file
    )

    gazebo_world_file_path = os.path.join(
        get_package_share_directory(package_name),
        "worlds",
        gazebo_world_file
    )

    # Read the URDF file content
    with open(urdf_file_path, 'r') as infp:
        robot_description = infp.read()

    # Define nodes
    file_server2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_file_server2, 'launch', 'ros_sharp_communication.launch.py')),
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': "-r " + gazebo_world_file_path}.items()
    )

    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=["-file", urdf_file_path, " -z ",  "1"]
        )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': True}, 
            {'robot_description': robot_description}
        ]
    )

    gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file_path
        }],
        output='screen'
    )

    gazebo_camera_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=["camera"],
        output='screen',
        parameters=[
                {'use_sim_time': True},
                {'qos_overrides./camera/compressed.depth': 10},
                {'qos_overrides./camera/compressed.reliability': 'best_effort'},
                {'qos_overrides./camera/compressed.durability': 'volatile'},
                {'qos_overrides./camera.depth': 10},
                {'qos_overrides./camera.reliability': 'best_effort'},
                {'qos_overrides./camera.durability': 'volatile'}
            ],
    )   

    joy_to_twist2_node = Node(
        package='gazebo_simulation_scene2',
        executable='joy_to_twist2',
        output='screen'
    )

    return [
        file_server2_node,
        gazebo_node,
        spawn_entity,
        gazebo_bridge_node,
        gazebo_camera_bridge_node,
        robot_state_publisher_node,
        joy_to_twist2_node
    ]

# Function to get the package share directory
def get_package_share_directory(package_name):
    from ament_index_python.packages import get_package_share_directory as get_pkg_share_dir
    return get_pkg_share_dir(package_name)
