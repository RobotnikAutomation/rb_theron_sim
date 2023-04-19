# Copyright (c) 2023, Robotnik Automation S.L.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Robotnik Automation S.L.L. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Robotnik Automation S.L.L. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os, launch, launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from robotnik_common.launch import add_launch_args

def generate_launch_description():

  ld = launch.LaunchDescription()
  p = [
    ('robot_id', 'Id of the robot', 'robot'),
    ('pos_x', 'X position of the robot', '0.0'),
    ('pos_y', 'Y position of the robot', '0.0'),
    ('pos_z', 'Z position of the robot', '0.1')
  ]
  params = add_launch_args(ld, p)

  # Node to spawn the robot in Gazebo
  robot_state_publisher = launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('rb_theron_gazebo'), 'launch', 'description.launch.py')
    ),
    launch_arguments={
      'environment': 'false',
      'use_sim_time': 'true',
      'robot_id': params['robot_id'],
    }.items(),
  )

  spawn_robot = launch_ros.actions.Node(
    namespace=params['robot_id'],
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
      '-entity', params['robot_id'],
      '-topic', 'robot_description',
      '-x', params['pos_x'],
      '-y', params['pos_y'],
      '-z', params['pos_z'],
    ],
  )

  # Run robot description and spawn robot in Gazebo
  ld.add_action(launch.actions.GroupAction(actions=[
    robot_state_publisher,
    spawn_robot,
  ]))

  # When spawn entity finishes, spawn controllers to avoid race condition.
  ld.add_action(launch.actions.RegisterEventHandler(
    event_handler=OnProcessExit(
      target_action=spawn_robot,
      on_exit=[
        launch_ros.actions.Node(
          namespace=params['robot_id'],
          package="controller_manager",
          executable="spawner",
          arguments=["robotnik_base_control"]
        ),
        launch_ros.actions.Node(
          namespace=params['robot_id'],
          package="controller_manager",
          executable="spawner",
          arguments=["joint_state_broadcaster"]
        ),
      ]
    )
  ))

  return ld
