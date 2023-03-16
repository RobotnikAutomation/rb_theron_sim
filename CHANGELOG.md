# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).


## [Unreleased]

### Added
- Changelog file
- Added rosbridge bringup
- Updated dockerfile with mainly format improvement
- Updated docker instructions
- Added environment variable to `X_INIT_POSE` to change default value of robot x init pose
- Added environment variable to `Y_INIT_POSE` to change default value of robot y init pose
- Added environment variable to `Z_INIT_POSE` to change default value of robot z init pose
- Added environment variable to `INIT_YAW` to change default value of robot yaw init pose
- Added environment variable to `LAUNCH_GMAPPING` to change default value of launching gmapping
- Added environment variable to `LAUNCH_AMCL` to change default value of launching amcl
- Added environment variable to `LAUNCH_MAPSERVER` to change default value of launching map_server
- Added environment variable to `LAUNCH_MOVE_BASE` to change default value of launching move_base
- Added environment variable to `LAUNCH_PAD` to change default value of launching joystick_pad
- Added environment variable to `LAUNCH_ROSBRIDGE` to change default value of launching rosbridge
- Added environment variable to `LAUNCH_RVIZ` to change default value of launching rviz
- Added environment variable to `USE_GPU` to change default value of gazebo use gpu
- Added environment variable to `VERBOSE` to change default value of gazebo verbose
- Added environment variable to `GUI` to change default value of gazebo gui launch
- Added environment variable to `DEBUG` to change default value of gazebo debug
- Added environment variable to `ROSBRIDGE_PORT` to change default value of default rosbridge port
### Changed
- Separated rviz launch from gazebo launch files
- Resorted arguments in order to group in a more sensible way
- Mounted dev folder on docker compose