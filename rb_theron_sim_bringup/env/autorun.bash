#!/bin/bash

export ROS_MASTER_URI=http://$HOSTNAME:11311
export ROS_HOSTNAME=$HOSTNAME

source ~/catkin_ws/devel/setup.bash

# RB1
# AUTOBOOT
echo "ROBOTNIK RB-Theron"
Terminal=`tty`
case $Terminal in
    "/dev/tty3") sleep 10;
    screen -S core -d -m roscore;
    sleep 15;
    screen -S bringup -d -m rosrun rosmon rosmon --name=rosmon_bringup rb_theron_sim_bringup rb_theron_complete.launch;
    sleep 5;
    screen -S map_nav_manager -d -m rosrun rosmon rosmon rb_theron_bringup map_nav_manager.launch;
    sleep 5;
    screen -S robotnik_hmi -d -m rosrun rosmon rosmon --name=robotnik_hmi robotnik_hmi robotnik_hmi.launch;
    sleep 2;
    screen -S rosbag_manager -d -m rosrun rosmon rosmon --name=rosmon_rosbag_manager rb_theron_bringup rosbag_manager.launch;
    sleep 2;
    screen -S command_manager -d -m rosrun rosmon rosmon --name=command_manager robot_simple_command_manager robot_simple_command_manager.launch;
    sleep 2;
    screen -S command_sequencer -d -m rosrun rosmon rosmon --name=command_sequencer robot_simple_command_sequencer robot_simple_command_sequencer.launch;
    sleep 2;
    screen -S complex_sequencer -d -m rosrun rosmon rosmon --name=complex_sequencer robot_complex_command_sequencer robot_complex_command_sequencer.launch;
    sleep 2;
    screen -S velocity_limiter -d -m roslaunch rb_theron_bringup robotnik_safety_controller.launch;

esac
