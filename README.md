# rb_theron_sim

=============

Packages for the simulation of the RB-Theron

<h1> Packages </h1>

<h2>rb_theron_gazebo</h2>

Launch files and world files to start the models in gazebo

<h2>rb_theron_sim_bringup</h2>

Launch files that launch the complete simulation of the robot



<h1>Simulating RB-Theron</h1>

### 1) Install the following dependencies:

This simulation has been tested using Gazebo 9 version. To facilitate the installation you can use the vcstool:

```bash
sudo apt-get install -y python3-vcstool
```

### 2) Create a workspace and clone the repository:

```bash
mkdir catkin_ws
cd catkin_ws
vcs import --input \
  https://raw.githubusercontent.com/RobotnikAutomation/rb_theron_sim/noetic-devel/repos/rb_theron_sim.repos
rosdep install --from-paths src --ignore-src -y -r
```

### 3) Compile:

```bash
catkin build
source devel/setup.bash
```


### 4) Launch RB-Theron simulation (1 robot by default, up to 3 robots):
- RB-Theron:

```bash
roslaunch rb_theron_sim_bringup rb_theron_complete.launch
```