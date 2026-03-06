#! /usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy import logging as ros_logger

from boxes_interfaces.msg import XboxInput, XboxOutput
from boxes_utils import VOLATILE_QOS


class XboxControllerInterface(Node):
    def __init__(self):
        super().__init__("xbox_control_interface")

        self.controller_read_timer = self.create_timer(0.02, self.read_controller)

        self.teleop_publisher = self.create_publisher(
            msg_type=XboxInput,
            topic="controller_input",
            qos_profile=VOLATILE_QOS,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.controller_feedback = self.create_subscription(
            msg_type=XboxOutput,
            topic="controller_feedback",
            qos_profile=VOLATILE_QOS,
            callback=MutuallyExclusiveCallbackGroup()
        )

    def read_controller(self):
        pass

    def write_controller(self):
        pass


def main(args=None):
    print("Starting Node...")
    rclpy.init(args=args)
    xbox_node = XboxControllerInterface()

    # TODO: think about what this should be? 
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(xbox_node)
    xbox_node.get_logger().info("Xbox Control Interface Initialized.")

    try:
        executor.spin()
    except KeyboardInterrupt:
        ros_logger.get_logger("executor_logger").warning(
            "Keyboard Interrupt - Shutting down Xbox Control Interface!"
        )
    

if __name__ == "__main__":
    main()