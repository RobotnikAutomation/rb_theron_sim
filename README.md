# RB-THERON simulation package

Packages for the simulation of the RB-Theron

- [RB-THERON simulation package](#rb-theron-simulation-package)
  - [Packages](#packages)
    - [rb\_theron\_gazebo](#rb_theron_gazebo)
    - [rb\_theron\_sim\_bringup](#rb_theron_sim_bringup)
  - [Simulating RB-Theron](#simulating-rb-theron)
    - [Install dependencies](#install-dependencies)
    - [Workspace and repository](#workspace-and-repository)
    - [Compile:](#compile)
    - [Launch RB-Theron simulation](#launch-rb-theron-simulation)
    - [Environment Variables](#environment-variables)
  - [Docker](#docker)
    - [Installation](#installation)
    - [Usage](#usage)
    - [Notes](#notes)

## Packages

### rb_theron_gazebo

Launch files and world files to start the models in gazebo

### rb_theron_sim_bringup

Launch files that launch the complete simulation of the robot

## Simulating RB-Theron

This are the steps to run the simulation in ros environment. 
It's assumed that already have installed the following on your system:

- [ROS](http://wiki.ros.org/ROS/Installation)
- [catkin-tools](https://pypi.org/project/catkin-tools/)
- [rosdep](https://pypi.org/project/rosdep/)

### Install dependencies

This simulation has been tested using Gazebo 9 version. To facilitate the installation you can use the vcstool:

```bash
sudo apt-get install -y python3-vcstool
```

### Workspace and repository

Create the workspace and clone the repository using the following commands:

```bash
mkdir catkin_ws
cd catkin_ws
vcs import --input \
  https://raw.githubusercontent.com/RobotnikAutomation/rb_theron_sim/melodic-devel/repos/rb_theron_sim.repos
rosdep install --from-paths src --ignore-src -y -r
```

### Compile:

You will need the compile the workspace in order to make it work

```bash
catkin build
source devel/setup.bash
```

### Launch RB-Theron simulation

For default configuration with 1 robot:

```bash
roslaunch rb_theron_sim_bringup rb_theron_complete.launch
```

if you want to use more than one robot specify the number using the parameter `robot_qty` or the environment varialble `ROBOT_QTY`:

```bash
roslaunch rb_theron_sim_bringup rb_theron_complete.launch robot_qty:=5
```

or:

```bash
ROBOT_QTY=5 roslaunch rb_theron_sim_bringup rb_theron_complete.launch
```

### Environment Variables

#### Robot quantity and namespace parameters

| Environment | Default Value | Meaning                               |
| ----------- | ------------- | ------------------------------------- |
| `ROBOT_QTY` | `1`           | Number of robots to spawn (maximum 5) |
| `ROBOT_ID`  | `robot`       | Robot namespace                       |

#### Single robot or robot 1 init pose parameters

| Environment   | Default Value | Meaning                                                    |
| ------------- | ------------- | ---------------------------------------------------------- |
| `X_INIT_POSE` | `0.0`         | robot or robot_1 (in case of multiple robot) x init pose   |
| `Y_INIT_POSE` | `0.0`         | robot or robot_1 (in case of multiple robot) y init pose   |
| `Z_INIT_POSE` | `0.0`         | robot or robot_1 (in case of multiple robot) z init pose   |
| `INIT_YAW`    | `0.0`         | robot or robot_1 (in case of multiple robot) yaw init pose |

#### Robot 2 init pose parameters

| Environment      | Default Value | Meaning               |
| ---------------- | ------------- | --------------------- |
| `R2_X_INIT_POSE` | `2.0`         | robot_2 x init pose   |
| `R2_Y_INIT_POSE` | `1.0`         | robot_2 y init pose   |
| `R2_Z_INIT_POSE` | `0.00`        | robot_2 z init pose   |
| `R2_INIT_YAW`    | `0.0`         | robot_2 yaw init pose |

#### Robot 3 init pose parameters

| Environment      | Default Value | Meaning               |
| ---------------- | ------------- | --------------------- |
| `R3_X_INIT_POSE` | `-2.0`        | robot_3 x init pose   |
| `R3_Y_INIT_POSE` | `-1.0`        | robot_3 y init pose   |
| `R3_Z_INIT_POSE` | `0.00`        | robot_3 z init pose   |
| `R3_INIT_YAW`    | `0.0`         | robot_3 yaw init pose |

#### Robot 4 init pose parameters

| Environment      | Default Value | Meaning               |
| ---------------- | ------------- | --------------------- |
| `R4_X_INIT_POSE` | `-2.0`        | robot_4 x init pose   |
| `R4_Y_INIT_POSE` | `1.0`         | robot_4 y init pose   |
| `R4_Z_INIT_POSE` | `0.00`        | robot_4 z init pose   |
| `R4_INIT_YAW`    | `0.0`         | robot_4 yaw init pose |

#### Robot 5 init pose parameters

| Environment      | Default Value | Meaning               |
| ---------------- | ------------- | --------------------- |
| `R5_X_INIT_POSE` | `2.0`         | robot_5 x init pose   |
| `R5_Y_INIT_POSE` | `-1.0`        | robot_5 y init pose   |
| `R5_Z_INIT_POSE` | `0.00`        | robot_5 z init pose   |
| `R5_INIT_YAW`    | `0.0`         | robot_5 yaw init pose |

#### Launch flags

| Environment               | Default Value | Meaning                                  |
| ------------------------- | ------------- | ---------------------------------------- |
| `LAUNCH_GMAPPING`         | `false`       | launching gmapping                       |
| `LAUNCH_AMCL`             | `true`        | launching amcl                           |
| `LAUNCH_MAPSERVER`        | `true`        | launching mapserver                      |
| `LAUNCH_MOVE_BASE`        | `true`        | launching move base                      |
| `LAUNCH_PAD`              | `true`        | launching pad (disabled for multirobots) |
| `LAUNCH_RVIZ`             | `true`        | launching rviz                           |
| `LAUNCH_LASER_MERGER`     | `false`       | launching laser_merger                   |
| `LAUNCH_WEB_BACKEND`      | `false`       | launching web backend                    |
| `LAUNCH_POSE_PUBLISHER`   | `false`       | launching pose publisher                 |
| `LAUNCH_WEB_THROTTLE`     | `false`       | launching web throttle                   |
| `LAUNCH_ROSBRIDGE`        | `true`        | launching rosbridge                      |
| `LAUNCH_WEB_PAD`          | `true`        | launching web pad                        |
| `LAUNCH_WEB_VIDEO_SERVER` | `true`        | launching web video server               |
| `USE_GPU`                 | `true`        | gazebo use gpu                           |

#### Gazebo parameters

| Environment | Default Value | Meaning           |
| ----------- | ------------- | ----------------- |
| `VERBOSE`   | `false`       | gazebo verbose    |
| `GUI`       | `false`       | gazebo gui launch |
| `DEBUG`     | `false`       | gazebo debug      |

#### Pose republisher parameters

| Environment                        | Default Value    | Meaning                      |
| ---------------------------------- | ---------------- | ---------------------------- |
| `POSE_PUBLISHER_FREQUENCY`         | `10`             | pose publisher frequency     |
| `POSE_PUBLISHER_BASE_FRAME_SUFFIX` | `base_footprint` | robot base frame suffix      |
| `POSE_PUBLISHER_TOPIC_REPUB`       | `pose`           | robot pose republished topic |

#### Rosbridge parameters

| Environment      | Default Value | Meaning |
| ---------------- | ------------- | ------- |
| `ROSBRIDGE_PORT` | `9090`        | default |

#### Web Throttle parameters

| Environment               | Default Value    | Meaning                     |
| ------------------------- | ---------------- | --------------------------- |
| `THROTTLE_MAP`            | `false`          | Throttle map                |
| `ROBOT_GPS_MODEL`         | `none`           | gps model to                |
| `ROBOT_HMI_2D_SENSOR_1`   | `none`           | 2d laser scan 1 to throttle |
| `ROBOT_HMI_2D_SENSOR_2`   | `none`           | 2d laser scan 2 to throttle |
| `ROBOT_HMI_2D_SENSOR_3`   | `none`           | 2d laser scan 3 to throttle |
| `ROBOT_HMI_3D_SENSOR_1`   | `none`           | 3d laser scan 1 to throttle |
| `ROBOT_HMI_3D_SENSOR_2`   | `none`           | 3d laser scan 2 to throttle |
| `ROBOT_HMI_3D_SENSOR_3`   | `none`           | 3d laser scan 3 to throttle |
| `ROBOT_HMI_MAP_NAV_TOPIC` | `navigation_map` | navigation map throttle     |
| `ROBOT_HMI_MAP_TOPIC`     | `map`            | map to throttle             |

#### Robot web joystick parameters

| Environment               | Default Value        | Meaning                                 |
| ------------------------- | -------------------- | --------------------------------------- |
| `ROBOT_WEB_PAD_PLUGIN`    | `web_pad`            | web pad configuration file              |
| `ROBOT_WEB_PAD_TOPIC`     | `web_joy`            | web pad joy toppic                      |
| `ROBOT_WEB_PAD_TWISTMUX`  | `web_teleop/cmd_vel` | topic to publish cmd_vel                |
| `ROBOT_WEB_VIDEO_QUALITY` | `50`                 | quality of transmission of video server |

#### Robot web video server parameters

| Environment                      | Default Value | Meaning                          |
| -------------------------------- | ------------- | -------------------------------- |
| `ROBOT_WEB_VIDEO_SERVER_ADDRESS` | `0.0.0.0`     | allowed ips for web video server |
| `ROBOT_WEB_VIDEO_SERVER_PORT`    | `8092`        | port for web video server        |
| `ROBOT_WEB_VIDEO_TRANSPORT`      | `compressed`  | type of video transport          |

## Docker

The simulation requires a graphical user interface. If you have an nvidia powered machine, you can use the nvidia-containers to use graphical acceleration and improve perfomance.

### Installation

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
&& ln -sf docker-compose-nvidia.yaml docker-compose.yaml \
|| ln -sf docker-compose-intel.yaml docker-compose.yaml
cd docker
docker compose up
```

**NOTE:** keep in mind that all changes on the rb-theron files, won't be updated to image if you do not rebuild image
if you want to make sure that every time you rebuild the container use the following command:

```bash
docker compose up --build
```

#### Manual Build

If you wish to build the image without launching the simulation use the following commands:

```bash
cd docker
docker compose build
```

### Notes

- This is docker requires a graphical interface

- The `ROS_MASTER_URI` is accessible outside the container, so in the host any ros command should work

- You could also run a `roscore` previous to launch the simulation in order to have some processes on the host running

- if you want to enter on the container use the following command in another terminal:
   ```bash
   docker exec -it docker-rb-theron-sim-1 bash
   ```
- In order to exit you have to 2 options:
1. Close `gazebo` and `rviz` and wait a bit

2. execute in another terminal in the same folder than the `docker-compose.yml`:
   
   ```bash
   docker compose down
   ```
