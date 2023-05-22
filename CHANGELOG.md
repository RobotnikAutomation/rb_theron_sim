# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).


## [Unreleased]

### Added
- Added [pose_publisher](https://github.com/RobotnikAutomation/pose_publisher) repository to `repos`file.
- Added environment variable to `LAUNCH_POSE_PUBLISHER` to launch the node_pose_publisher. 
- Added environment variable to `POSE_PUBLISHER_FREQUENCY` to change default value of the publishing frequency for the node_pose_publisher.
- Added environment variable to `POSE_PUBLISHER_MAP_FRAME` to change default value of robot map frame.
- Added environment variable to `POSE_PUBLISHER_BASE_FRAME` to change default value of robot base frame.
- Added environment variable to `POSE_PUBLISHER_TOPIC_REPUB` to change default value of the topic name where the node_pose_publisher publishes.
### Changed
- Modified the `rb_theron_complete.launch` to add the pose_publisher `.launch` and read the environment variables to configure it.
- Modified the `rb_theron_gazebo.launch` to launch and configure the pose_publisher node.
- Modified the `rb_theron_robot.launch` to launch and configure the pose_publisher node.
- Updated the docker-compose `.yaml` (`nvidia` and `intel`) to add and configure the default values of the new environment variables.
- Updated `README.md` file.
  - Update environment variables.
  - Updated docker usage.
- Updated `CHANGELOG.md` file.