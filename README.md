# About this fork

Add support for gazebo classic. 

The modifications are: 

* add ros2 control setup for gazebo
* add grasp fix plugin
* fix ["_mimic" joint interface name suffix causes Moveit2 "Joint not found" Error](https://github.com/ros-controls/gazebo_ros2_control/issues/173)
* add moveit2 config
* provide a launch file example to launch robotiq 2f 85 gripper in gazebo



If you use prefix in robotiq, remember to change the move group name and joint name in those yaml files (files under `robotiq_description/config`)manually. 



Test grasp action for robotiq_2f_85

```shell
ros2 run robotiq_description test_grpper --ros-args -p gripper_position:=0.5 -p use_sim_time:=true
```







------

The following is from the original repo: 

# ros2_robotiq_gripper

This repository contains the ROS 2 driver, controller and description packages for working with a Robotiq Gripper.


## Build status



ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`rolling`](https://github.com/PickNikRobotics/ros2_robotiq_gripper/tree/rolling) | [![Rolling Binary Build](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/rolling-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/rolling-binary-build-main.yml?branch=main) <br /> [![Rolling Semi-Binary Build](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/rolling-semi-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/rolling-semi-binary-build-main.yml?branch=main) | [![Doxygen Doc Deployment](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/doxygen-deploy.yml/badge.svg)](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/doxygen-deploy.yml) <br /> [Generated Doc](https://PickNikRobotics.github.io/ros2_robotiq_gripper_Documentation/rolling/html/index.html) | [ros2_robotiq_gripper](https://index.ros.org/p/ros2_robotiq_gripper/#rolling)


ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Humble** | [`humble`](https://github.com/PickNikRobotics/ros2_robotiq_gripper/tree/humble) | [![Humble Binary Build](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/humble-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/humble-binary-build-main.yml?branch=main) <br /> [![Humble Semi-Binary Build](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/humble-semi-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/humble-semi-binary-build-main.yml?branch=main) | [![Doxygen Doc Deployment](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/doxygen-deploy.yml/badge.svg)](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/doxygen-deploy.yml) <br /> [Generated Doc](https://PickNikRobotics.github.io/ros2_robotiq_gripper_Documentation/humble/html/index.html) | [ros2_robotiq_gripper](https://index.ros.org/p/ros2_robotiq_gripper/#humble)


ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Iron** | [`iron`](https://github.com/PickNikRobotics/ros2_robotiq_gripper/tree/iron) | [![Iron Binary Build](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/iron-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/iron-binary-build-main.yml?branch=main) <br /> [![Iron Semi-Binary Build](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/iron-semi-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/iron-semi-binary-build-main.yml?branch=main) | [![Doxygen Doc Deployment](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/doxygen-deploy.yml/badge.svg)](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/doxygen-deploy.yml) <br /> [Generated Doc](https://PickNikRobotics.github.io/ros2_robotiq_gripper_Documentation/iron/html/index.html) | [ros2_robotiq_gripper](https://index.ros.org/p/ros2_robotiq_gripper/#iron)

### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

[Detailed build status](.github/workflows/README.md)

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `$NAME$.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.
