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
        self.new_joint_commands = JointCommandArray()
        self.last_joint_commmands = {}
        # Continuously updated dictionary with the last veloctity joint data.
        self.prev_vel_joint_states = {}

        self.declare_parameter('update_rate', 10.0)
        self.update_rate = self.get_parameter('update_rate').value

    def joint_cmds_callback(self, data):
        """Save current joint commands."""
        self.new_joint_commands = data

        # Save new joint command into dict with last joint commands data
        for joint_command in self.new_joint_commands.joint_command_array:
            self.last_joint_commmands[joint_command.name] = joint_command

    def update_joint_states(self):
        """Process joint commands and send the simulated joint states."""
        # Only process continue if data has already arrived
        if len(self.new_joint_commands.joint_command_array) == 0:
            return

        joint_msg = JointState()

        # Loop through all joint commands
        for names, joint_command in self.last_joint_commmands.items():
            # Handle steering and deployment joints
            if joint_command.mode == 'POSITION':
                # Set commanded position as joint state
                joint_msg.header.stamp = super().get_clock().now().to_msg()

                joint_msg.name.append(joint_command.name)
                joint_msg.position.append(joint_command.value)
                joint_msg.effort.append(0.0)
                joint_msg.velocity.append(0.0)

            if joint_command.mode == 'VELOCITY':
                # Initialize joint state if the joint is new.
                if joint_command.name not in self.prev_vel_joint_states:
                    # Add joint to prev_vel_joint_states in case it was not added before
                    self.prev_vel_joint_states[joint_command.name] = {}

                    self.prev_vel_joint_states[
                        joint_command.name]['stamp'] = super().get_clock().now().to_msg()
                    self.prev_vel_joint_states[joint_command.name]['position'] = 0.0
                    self.prev_vel_joint_states[joint_command.name]['effort'] = 0.0
                    # velocity is set to zero since no motion has taken place yet
                    self.prev_vel_joint_states[joint_command.name]['velocity'] = 0.0

                # Calculate new joint position by integrating the velocity command
                else:
                    joint_msg.header.stamp = super().get_clock().now().to_msg()

                    # Get time in seconds and nanoseconds tuple (sec, nanosec)
                    time_s_n = super().get_clock().now().seconds_nanoseconds()

                    # Compute time from last command to current time in seconds [s]
                    delta_t = time_s_n[0] - self.prev_vel_joint_states[joint_command.name]['stamp'].sec + \
                       round((time_s_n[1] - self.prev_vel_joint_states[joint_command.name]['stamp'].nanosec)/pow(10, 9), 3)

                    # Compute new position with velocity*d_t + position_previos
                    new_position = joint_command.value * delta_t + self.prev_vel_joint_states[
                        joint_command.name]['position']

                    # Update previous velocity joint states
                    self.prev_vel_joint_states[
                        joint_command.name]['position'] = new_position
                    self.prev_vel_joint_states[
                        joint_command.name]['stamp'] = joint_msg.header.stamp
                    self.prev_vel_joint_states[
                        joint_command.name]['effort'] = 0.0
                    self.prev_vel_joint_states[
                        joint_command.name]['velocity'] = joint_command.value

                    # Populate new message
                    joint_msg.name.append(joint_command.name)
                    joint_msg.position.append(new_position)
                    joint_msg.effort.append(0.0)
                    joint_msg.velocity.append(joint_command.value)

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
