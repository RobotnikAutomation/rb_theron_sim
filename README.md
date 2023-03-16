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
  https://raw.githubusercontent.com/RobotnikAutomation/rb_theron_sim/melodic-devel/repos/rb_theron_sim.repos
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

### Environment Variables

| Enviroment         | Default Value | Meaning              |
| ------------------ | ------------- | -------------------- |
| `X_INIT_POSE`      | `0.0`         | robot x init pose    |
| `Y_INIT_POSE`      | `0.0`         | robot y init pose    |
| `Z_INIT_POSE`      | `0.15`        | robot z init pose    |
| `INIT_YAW`         | `0.0`         | robot yaw init pose  |
| `LAUNCH_GMAPPING`  | `false`       | launching gmapping   |
| `LAUNCH_AMCL`      | `true`        | launching amcl       |
| `LAUNCH_MAPSERVER` | `true`        | launching map server |
| `LAUNCH_MOVE_BASE` | `true`        | launching move base  |
| `LAUNCH_PAD`       | `true`        | launching joystick   |
| `LAUNCH_ROSBRIDGE` | `true`        | launching rosbridge  |
| `LAUNCH_RVIZ`      | `true`        | launching rviz       |
| `USE_GPU`          | `true`        | gazebo use gpu       |
| `VERBOSE`          | `false`       | gazebo verbose       |
| `GUI`              | `true`        | gazebo gui launch    |
| `DEBUG`            | `false`       | gazebo debug         |
| `ROSBRIDGE_PORT`   | `9090`        | default              |

## Docker Usage

In order to run this simulation you will need nvidia graphical accelation

### Installation of required files

#### Intel GPU

- [docker engine](https://docs.docker.com/engine/install/ubuntu/)
- [docker compose plugin](https://docs.docker.com/compose/install/linux/)

#### Nvidia GPU

- [docker engine](https://docs.docker.com/engine/install/ubuntu/)

- [docker compose plugin](https://docs.docker.com/compose/install/linux/)

- nvidia-drivers

- [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

### Usage

```bash
git clone https://github.com/RobotnikAutomation/rb_theron_sim.git
cd rb_theron_sim
git checkout melodic-devel
nvidia-smi &>/dev/null \
&& ln -sf docker-compose-nvidia.yml docker-compose.yml \
|| ln -sf docker-compose-intel.yml docker-compose.yml
cd docker
docker compose up
```

#### Manual Build

If you wish to build the image without launching the simulation use the following commands:

```bash
cd docker
docker compose build
```

#### Notes

This is docker requires a graphical interface

- In order to exit you have to 2 options

- The `ROS_MASTER_URI` is accessible outside the container, so in the host any ros command should work

- You could also run a `roscore` previous to launch the simulation in order to have some processes on the host running

- if you want to enter on the container use the following command in another terminal
1. Close `gazebo` and `rviz` and wait a bit

2. execute in another terminal:
   
   ```bash
   docker container rm --force docker-rb-theron-sim-1
   ```

#### Notes

- This is docker requires a graphical interface

- The `ROS_MASTER_URI` is accessible outside the container, so in the host any ros command should work

- You could also run a `roscore` previous to launch the simulation in order to have some processes on the host running

- if you want to enter on the container use the following command in another terminal
  
  ```bash
  docker container exec -it docker-rb-theron-sim-1 bash
  ```

- In order to exit you have to 2 options
1. Close `gazebo` and `rviz` and wait a bit

2. execute in another terminal in the same folder than the `docker-compose.yml`:
   
   ```bash
   docker compose down
   ```


