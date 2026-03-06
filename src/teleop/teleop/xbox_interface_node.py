#! /usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy import logging as ros_logger


class XboxControllerInterface(Node):
    def __init__(self):
        super().__init__("xbox_control_interface")
        # TODO: we probably want 2 timers 1 which reads events from xbox dongle,
        # compiles them into a state object at about 200Hz
        # then another which publishes those events reading from that local state object

        # TODO: add a topic with a specific message type


def main(args=None):
    print("Starting Node...")
    rclpy.init(args=args)
    xbox_node = XboxControllerInterface()
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