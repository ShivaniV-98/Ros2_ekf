# Copyright 2023 Nick Morales.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, Shutdown
from launch.substitutions import Command, TextSubstitution, \
    PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    """Launch robot description and optionally RVIZ."""
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Selects whether or not to launch RVIZ.',
        ),
        DeclareLaunchArgument(
            name='use_jsp',
            default_value='true',
            choices=['true', 'false'],
            description='Selects whether or not to launch the joint state publisher.',
        ),
        DeclareLaunchArgument(
            name='color',
            default_value='purple',
            choices=['red', 'green', 'blue', 'purple'],
            description='Selects the color of the turtlebot.',
        ),
        SetLaunchConfiguration(
            name='tf_prefix',
            value=[
                LaunchConfiguration('color'),
                TextSubstitution(text='/'),
            ]
        ),
        SetLaunchConfiguration(
            name='rviz_config_filename',
            value=[
                TextSubstitution(text='basic_'),
                LaunchConfiguration('color'),
                TextSubstitution(text='.rviz'),
            ]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=LaunchConfiguration('color'),
            parameters=[{
                'robot_description':
                    ParameterValue(
                        Command([
                            TextSubstitution(text='xacro '),
                            PathJoinSubstitution([
                                FindPackageShare('nuturtle_description'),
                                'urdf/turtlebot3_burger.urdf.xacro'
                            ]),
                            TextSubstitution(text=' color:='),
                            LaunchConfiguration('color'),
                        ]),
                        value_type=str
                    ),
                'frame_prefix':
                    LaunchConfiguration('tf_prefix'),
            }],
            remappings=[
                ('joint_states', '/joint_states')
            ]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=LaunchConfiguration('color'),
            condition=IfCondition(LaunchConfiguration('use_jsp')),
            remappings=[
                ('joint_states', '/joint_states')
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            namespace=LaunchConfiguration('color'),
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('nuturtle_description'),
                    'config',
                    LaunchConfiguration('rviz_config_filename')
                ]),
                '-f',
                [
                    LaunchConfiguration('tf_prefix'),
                    TextSubstitution(text='base_footprint'),
                ]
            ],
            on_exit=Shutdown()
        ),
    ])
