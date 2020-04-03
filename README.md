# Simple Joint Simulation

## Overview

This packages provides a very simple simulation of joints for visiualization of joint states in RVIZ. It is designed to work in conjunction with [joint_state_publisher](http://wiki.ros.org/joint_state_publisher), the [robot_state_publisher](http://wiki.ros.org/robot_state_publisher) and the joint_command message defined in [rover_msgs].

It takes joint commands as inputs and outputs joint states. It differentiates between position and velocity controlled joints.

Velocity controlled joints are currently only simulated in the joint command callback. Thus, a constant stream of joint commands is required to keep the joint moving in the desired velocity.

**Keywords:** joint, states, simulation, package

### License

The source code is released under a [TODO: Add License]().

**Author: Miro Voellmy<br />
Affiliation: [European Space Agency](https://www.esa.int/)<br />
Maintainer: Miro Voellmy, miro.voellmy@esa.com**

The Simple Joint Simulation package has been tested under [ROS2] Eloquent and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- ([rover_msgs]) (message definitions for ESA-PRL rovers)

#### Building

To build from source, clone the latest version from this repository into your ros2 workspace and compile the package using

	cd ros2_ws/src
	git clone https://github.com/esa-prl/simple_joint_simulation
	cd ../
	colcon build

## Usage

Run the main node with

	ros2 run simple_joint_simulation simple_joint_simulation_node

See [marta_launch/launch/simple_simulation.launch.py](https://github.com/esa-prl/marta_launch/blob/master/launch/simple_simulation.launch.py) for use in context.

## Nodes

### simple_joint_simulation_node

Outputs joint states based on joint commands.

#### Subscribed Topics

* **`/rover_joint_cmds`** ([rover_msgs/JointCommandArray])

	The desired joint states.


#### Published Topics

* **`/joint_states_sim`** ([sensor_msgs/JointState])

	The simulated joint states which shall be added as a joint state source to the `joint_state_publisher`. The `joint_state_publisher` then publishes the `/joint_states` topic containint all joint states.

## Bugs & Feature Requests

Please report bugs and request features using the github issue tracker.

[ROS2]: http://www.ros.org
[rover_msgs]: https://github.com/esa-prl/rover_msgs
[rover_msgs/JointCommandArray]: https://github.com/esa-prl/rover_msgs/blob/master/msg/JointCommandArray.msg
[rviz]: http://wiki.ros.org/rviz
[sensor_msgs/JointState]: http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
