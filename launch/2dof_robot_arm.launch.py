# Copyright (c) 2020 OUXT Polaris
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

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

from pathlib import Path


def generate_launch_description():
    share_dir_path = os.path.join(get_package_share_directory('dynamixel_hardware_interface'))
    xacro_path = os.path.join(
        share_dir_path,
        'config',
        'urdf',
        '2dof_robot_arm_robot.urdf.xacro')

    doc = xacro.process_file(xacro_path)
    robot_description = {"robot_description": doc.toxml()}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description])
    view_model = LaunchConfiguration('view_model', default=False)
    view_model_arg = DeclareLaunchArgument(
                'view_model', default_value=view_model,
                description="if true, launch Autoware with given rviz configuration.")
    rviz = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output={
                    'stderr': 'log',
                    'stdout': 'log',
                    },
                condition=IfCondition(view_model),
                arguments=[
                    '-d', str(
                        Path(get_package_share_directory('dynamixel_hardware_interface')) /
                        'config' /
                        '2dof_robot_arm.rviz')])
    controller_config = os.path.join(
        get_package_share_directory("dynamixel_hardware_interface"),
        "config",
        "controllers",
        "controllers_2dof_robot_arm.yaml"
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    return launch.LaunchDescription(
        [
            robot_state_publisher,
            view_model_arg,
            rviz,
            control_node,
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "control",
                    "load_controller",
                    "joint_state_controller"],
                output="screen",
                shell=True,
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "control",
                    "load_controller",
                    "velocity_controller"],
                output="screen",
                shell=True,
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "control",
                    "load_controller",
                    "joint_trajectory_controller"],
                output="screen",
                shell=True,
            )
        ]
    )
