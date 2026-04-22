import rclpy
from rclpy import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy import logging as ros_logger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


from boxes_interfaces.msg import XboxVelocityInput
from boxes_utils import VOLATILE_QOS


class MotorControlInterface(Node):
    def __init__(self):
        super().__init__("motor_control")

        self.teleop_subscriber = self.create_subscription(
            msg_type=XboxVelocityInput,
            topic="controller_move_state",
            qos_profile=VOLATILE_QOS,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )


def main(args=None):
    print("Starting Motor Control Node...")
    rclpy.init(args=args)
    motor_control = MotorControlInterface()

    executor = SingleThreadedExecutor()
    executor.add_node(motor_control)
    motor_control.get_logger().info("Motor Control Node Intialized.")

    try:
        executor.spin()
    except KeyboardInterrupt:
        ros_logger.get_logger("executor_logger").warning(
            "Keyboard Interrupt - Shutting down Motor Control"
        )


if __name__ == "__main__":
    main()
