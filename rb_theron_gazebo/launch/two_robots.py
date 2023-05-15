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

import launch, launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from robotnik_common.launch import add_launch_args

def generate_launch_description():

  ld = launch.LaunchDescription()
  p = [
    ('verbose', 'Verbose output', 'false'),
    ('package_gazebo', 'Package name of the gazebo world', 'rb_theron_gazebo'),
    ('gazebo_world', 'Name of the gazebo world', 'default'),
    # First robot to spawn
    ('robot_id', 'Id of the robot', 'robot'),
    ('robot_description_file', 'URDF file to load', 'default.urdf.xacro'),
    ('pos_x', 'X position of the robot', '0.0'),
    ('pos_y', 'Y position of the robot', '0.0'),
    ('pos_z', 'Z position of the robot', '0.1')
  ]
  params = add_launch_args(ld, p)

  # Launch gazebo with the world
  ld.add_action(launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [launch_ros.substitutions.FindPackageShare(params['package_gazebo']), '/launch/gazebo.launch.py']
    ),
    launch_arguments={
      'verbose': params['verbose'],
      'world_name': params['gazebo_world'],
    }.items()
  ))

  # Spawn the robot a
  ld.add_action(launch.actions.GroupAction([
    launch.actions.IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        [launch_ros.substitutions.FindPackageShare('rb_theron_gazebo'), '/launch/spawn.launch.py']
      ),
      launch_arguments={
        'robot_id': 'robot_a',
        'robot_description_file': 'dual_laser.urdf.xacro',
        'pos_x': '-1.0',
        'pos_y': '0.0',
        'pos_z': '0.1',
      }.items(),
    )
  ]))
  # Spawn the robot b
  ld.add_action(launch.actions.GroupAction([
    launch.actions.IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        [launch_ros.substitutions.FindPackageShare('rb_theron_gazebo'), '/launch/spawn.launch.py']
      ),
      launch_arguments={
        'robot_id': 'robot_b',
        'robot_description_file': 'dual_laser.urdf.xacro',
        'pos_x': '0.5',
        'pos_y': '0.86',
        'pos_z': '0.1',
      }.items(),
    )
  ]))

  return ld
