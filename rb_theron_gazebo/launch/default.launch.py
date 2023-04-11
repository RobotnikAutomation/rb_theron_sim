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

import launch, launch_ros, os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def read_params(ld : launch.LaunchDescription, params : list[tuple[str, str, str]]): # name, description, default_value

  # Declare the launch options
  for param in params:
    ld.add_action(launch.actions.DeclareLaunchArgument(
      name=param[0], description=param[1], default_value=param[2],))

  # Get the launch configuration variables
  ret={}
  for param in params:
    ret[param[0]] = launch.substitutions.LaunchConfiguration(param[0])

  return ret


def generate_launch_description():

  ld = launch.LaunchDescription()
  p = [
    ('verbose', 'Verbose output', 'false'),
    ('package_gazebo', 'Package name of the gazebo world', 'opil_factory_world'),
    ('robots_n', 'Number of robots', '1'),
  ]
  params = read_params(ld, p)

  ld.add_action(launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [launch_ros.substitutions.FindPackageShare(params['package_gazebo']), '/launch/gazebo.launch.py']
    ),
    launch_arguments={
      'environment': 'false',
      'verbose': params['verbose'],
    }.items()
  ))

  ld.add_action(launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [launch_ros.substitutions.FindPackageShare('rb_theron_gazebo'), '/launch/spawn.launch.py']
    ),
    launch_arguments={
      'environment': 'false',
      'use_sim_time': 'true',
      'robot_id': 'robot_a',
      'namespace': 'robot_a',
      'pos_x': '0.0',
      'pos_y': '3.0',
      'pos_z': '0.1',
    }.items(),
  ))

  return ld
