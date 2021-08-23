# rb_theron_sim

=============

Packages for the simulation of the RB-Theron

<h1> Packages </h1>

<h2>rb_theron_gazebo</h2>

Launch files and world files to start the models in gazebo

<h2>rb_theron_sim_bringup</h2>

Launch files that launch the complete simulation of the robot



<h1>Simulating RB-Theron</h1>

1. Install the following dependencies:
  - rb_theron_common [link](https://github.com/RobotnikAutomation/rb_theron_common) (branch: melodic-devel)
  - robotnik_msgs [link](https://github.com/RobotnikAutomation/robotnik_msgs) (branch: master)
  - robotnik_sensors [link](https://github.com/RobotnikAutomation/robotnik_sensors) (branch: melodic-devel)

    In the workspace install the packages dependencies:
    ```
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -yr
    ```  

2. Launch RB-Theron simulation:
  ```
  roslaunch rb_theron_sim_bringup rb_theron_complete.launch
  ```

  Optional arguments:
  ```
  <arg name="id_robot" default="robot"/>
  <arg name="launch_robot" default="true"/>
  <arg name="xacro_robot" value="rb_theron.urdf.xacro"/>
  <arg name="x_init_pose" default="0.0" />
  <arg name="y_init_pose" default="0.0" />
  <arg name="z_init_pose" default="0.15" />
  <arg name="init_yaw" default="0.0" />
  <arg name="gmapping" default="false"/>
  <arg name="amcl" default="true"/>
  <arg name="mapserver" default="true"/>
  <arg name="map_frame" default="$(arg id_robot)_map"/>
  <arg name="map_file" default="$(find rb_theron_localization)/maps/willow_garage/willow_garage.yaml"/>
  <arg name="move_base" default="true"/>
  <arg name="pad" default="true"/>
  ```

3. Enjoy! You can use the topic "${id_robot}/robotnik_base_control/cmd_vel" to control the Rb-Theron robot or send simple goals using "/${id_robot}/move_base_simple/goal"
