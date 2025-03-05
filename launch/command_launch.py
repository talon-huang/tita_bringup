#!/usr/bin/python3
#
# Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
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

from launch_ros.actions import Node
import os
import sys

from launch.actions import IncludeLaunchDescription , RegisterEventHandler,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart

sys.path.insert(0, os.path.join(get_package_share_directory('tita_bringup'), 'launch'))
from launch_utils import *


def generate_launch_description():

    # Define the nodes to be launched
    teleop_cmd_node = return_launch_file('teleop_command','launch','teleop_command.launch.py')

    active_cmd_node = return_launch_file('active_command','launch','active_command.launch.py')

    # passive_cmd_node = return_launch_file('passive_command','launch','passive_command.launch.py')
    passive_cmd_node = TimerAction(
        period=1.0,
        actions=[return_launch_file('passive_command', 'launch', 'passive_command.launch.py')]
    )

    cmd_manager_node = return_launch_file('command_manager','launch','command_manager.launch.py')
    
    ld = LaunchDescription()
    # # Add independent nodes to LaunchDescription
    ld.add_action(teleop_cmd_node)
    ld.add_action(active_cmd_node)
    ld.add_action(passive_cmd_node)
    ld.add_action(cmd_manager_node)
    # Add dependent nodes using wait_and_launch
    # ld.add_action(RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=active_cmd_node,
    #         on_start=[cmd_manager_node]
    #     )
    # ))
    # Use the custom function to create the sequential launch description
    return ld