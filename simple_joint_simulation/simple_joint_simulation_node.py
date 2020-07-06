#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from rover_msgs.msg import JointCommandArray

from sensor_msgs.msg import JointState


class SimpleJointSimulation(Node):
    """Simulates a joint for every joint command received."""

    def __init__(self):
        """Initialize the node."""
        self.node_name = 'simple_joint_simulation_node'
        super().__init__(self.node_name)

        # Init Params
        self.init_params()

        # Create Publishers
        self.joint_states_pub_ = self.create_publisher(JointState, 'joint_states_sim', 10)

        # Create Subscriptions
        self.create_subscription(JointCommandArray, 'joint_cmds', self.joint_cmds_callback, 10)

        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))

    def init_params(self):
        """Initialize the member variables."""
        self.curr_data = JointCommandArray()
        # Continuously updated dictionary with the last veloctity joint data.
        self.prev_vel_joint_states = {}

        self.declare_parameter('update_rate', 10.0)
        self.update_rate = self.get_parameter('update_rate').value

    def joint_cmds_callback(self, data):
        """Save current joint commands."""
        self.curr_data = data

    def update_joint_states(self):
        """Process joint commands and send the simulated joint states."""
        # Only process data has already arrived
        if len(self.curr_data.joint_command_array) == 0:
            return

        joint_msg = JointState()

        # Loop through all joint commands
        for joint_command in self.curr_data.joint_command_array:

            # Handle steering and deployment joints
            if joint_command.mode == 'POSITION':
                # Set commanded position as joint state
                joint_msg.header.stamp = super().get_clock().now().to_msg()

                joint_msg.name.append(joint_command.name)
                joint_msg.position.append(joint_command.value)
                joint_msg.effort.append(0.0)
                joint_msg.velocity.append(0.0)

            if joint_command.mode == 'VELOCITY':
                # Check if previous information about joint is present:
                if joint_command.name not in self.prev_vel_joint_states:
                    # Add joint to prev_vel_joint_states in case it was not added before
                    self.prev_vel_joint_states[joint_command.name] = {}

                    self.prev_vel_joint_states[
                        joint_command.name]['stamp'] = super().get_clock().now().to_msg()
                    self.prev_vel_joint_states[joint_command.name]['position'] = 0.0
                    self.prev_vel_joint_states[joint_command.name]['effort'] = 0.0
                    # velocity is set to zero since no motion has taken place yet
                    self.prev_vel_joint_states[joint_command.name]['velocity'] = 0.0

                else:
                    joint_msg.header.stamp = super().get_clock().now().to_msg()

                    # This is how to get ros2 time now:
                    # s_n = super().get_clock().now().seconds_nanoseconds()
                    # Get time in seconds and nanoseconds tuple ()
                    time_s_n = super().get_clock().now().seconds_nanoseconds()

                    # Compute time from last command to current time in seconds [s]
                    delta_t = time_s_n[0] - self.prev_vel_joint_states[joint_command.name]['stamp'].sec + \
                       round((time_s_n[1] - self.prev_vel_joint_states[joint_command.name]['stamp'].nanosec)/pow(10, 9), 3)

                    # Only  set the new position and velocity if the command came within half a sec
                    if delta_t < 0.5:
                        # Compute new position with velocity*d_t + position_previos
                        new_position = joint_command.value * delta_t + self.prev_vel_joint_states[
                            joint_command.name]['position']

                        # Populate new message
                        joint_msg.name.append(joint_command.name)
                        joint_msg.position.append(new_position)
                        joint_msg.effort.append(0.0)
                        joint_msg.velocity.append(joint_command.value)

                        self.prev_vel_joint_states[joint_command.name]['position'] = new_position
                    else:
                        self.prev_vel_joint_states[joint_command.name]['position'] = 0.0

                    self.prev_vel_joint_states[
                        joint_command.name]['stamp'] = joint_msg.header.stamp
                    self.prev_vel_joint_states[joint_command.name]['effort'] = 0.0
                    self.prev_vel_joint_states[
                        joint_command.name]['velocity'] = joint_command.value

        self.joint_states_pub_.publish(joint_msg)

    def spin(self):
        """Proccess all callbacks in spin_once."""
        while rclpy.ok():
            self.update_joint_states()
            rclpy.spin_once(self, timeout_sec=1.0/self.update_rate)


    def stop(self):
        """Shutdown proceedure."""
        rclpy.loginfo('{} STOPPED.'.format(self.node_name.upper()))


def main(args=None):
    """Shutdown proceedure."""
    rclpy.init(args=args)

    simple_joint_simulation = SimpleJointSimulation()

    simple_joint_simulation.spin()

    # TODO: catch CTRL+C exception and close node gracefully.
    simple_joint_simulation.stop()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simple_joint_simulation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
