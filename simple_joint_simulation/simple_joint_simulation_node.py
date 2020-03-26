#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from rover_msgs.msg import JointCommand
from rover_msgs.msg import JointCommandArray

from sensor_msgs.msg import JointState

class SimpleJointSimulation(Node):
    def __init__(self):
        # Init Node
        self.node_name = 'simple_joint_simulation_node'
        super().__init__(self.node_name)

        # Init Params
        self.init_params()

        # Create Publishers        
        self.joint_states_pub_ = self.create_publisher(JointState, 'joint_states_sim', 10)

        # Create Subscriptions
        self.create_subscription(JointCommandArray, 'rover_joint_cmds', self.joint_cmds_callback, 10)

    def init_params(self):
        # Previous Joint Command Data - This might not contain data of all joints
        self.prev_data = JointCommandArray()
        # Continuously updated dictionary with the last veloctity joint data.
        self.prev_vel_joint_states = {}

    def joint_cmds_callback(self, data):

        self.curr_data = data

        # Initialize the prev_data message the first time.
        if len(self.prev_data.joint_command_array) == 0:
            self.prev_data = data
            return

        joint_msg = JointState()

        # Loop through all joint commands
        for joint_command in self.curr_data.joint_command_array:

            # Handle steering and deployment joints
            if joint_command.mode == "POSITION":          
                # Set commanded position as joint state
                joint_msg.header.stamp = super().get_clock().now().to_msg()

                joint_msg.name.append(joint_command.name)
                joint_msg.position.append(joint_command.value)
                joint_msg.effort.append(0.0)
                joint_msg.velocity.append(0.0)

            if joint_command.mode == "VELOCITY":
                # Check if previous information about joint is present:
                if joint_command.name not in self.prev_vel_joint_states:
                    # Add joint to prev_vel_joint_states in case it was not added before
                    self.prev_vel_joint_states[joint_command.name] = {}

                    self.prev_vel_joint_states[joint_command.name]['stamp'] = super().get_clock().now().to_msg()
                    self.prev_vel_joint_states[joint_command.name]['position'] = 0.0
                    self.prev_vel_joint_states[joint_command.name]['effort'] = 0.0
                    # velocity is set to zero since no motion has taken place yet
                    self.prev_vel_joint_states[joint_command.name]['velocity'] = 0.0

                else:
                    joint_msg.header.stamp = super().get_clock().now().to_msg()
                    
                    # This is how to get ros2 time now:
                    # s_n = super().get_clock().now().seconds_nanoseconds()

                    # Compute time from last command to new command in seconds [s]
                    delta_t = joint_msg.header.stamp.sec  - self.prev_vel_joint_states[joint_command.name]['stamp'].sec  + \
                       round((joint_msg.header.stamp.nanosec - self.prev_vel_joint_states[joint_command.name]['stamp'].nanosec)/pow(10,9),3)

                    # Only populate set the new position and velocity if the command came within half a second
                    if delta_t < 0.5:
                        new_position = joint_command.value*delta_t + self.prev_vel_joint_states[joint_command.name]['position']

                        # Populate new message
                        joint_msg.name.append(joint_command.name)
                        joint_msg.position.append(new_position)
                        joint_msg.effort.append(0.0)
                        joint_msg.velocity.append(joint_command.value)
                        
                        self.prev_vel_joint_states[joint_command.name]['position'] = new_position
                    else:
                        self.prev_vel_joint_states[joint_command.name]['position'] = 0.0
                    
                    self.prev_vel_joint_states[joint_command.name]['stamp'] = joint_msg.header.stamp
                    self.prev_vel_joint_states[joint_command.name]['effort'] = 0.0
                    self.prev_vel_joint_states[joint_command.name]['velocity'] = joint_command.value




        self.joint_states_pub_.publish(joint_msg)


    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)

    def stop(self):
        rospy.loginfo("{} STOPPED.".format(self.node_name.upper()))


def main(args=None):
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
