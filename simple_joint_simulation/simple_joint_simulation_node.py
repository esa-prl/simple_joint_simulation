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
        self.joint_states_pub_ = self.create_publisher(JointState, 'joint_states', 10)

        # Create Subscriptions
        self.create_subscription(JointCommandArray, 'rover_joint_cmds', self.joint_cmds_callback, 10)

    def init_params(self):
        self.prev_data = JointCommandArray()

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
                joint_msg.header.stamp = super().get_clock().now().to_msg()

                joint_msg.name.append(joint_command.name)
                joint_msg.position.append(joint_command.value)
                joint_msg.effort.append(0.0)

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
