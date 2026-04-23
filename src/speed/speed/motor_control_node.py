import rclpy
from rclpy import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy import logging as ros_logger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from boxes_interfaces.msg import XboxVelocityInput
from boxes_utils.qos import VOLATILE_QOS

from speed.vesc_uart import VescCommandInterface

UART_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200
LEFT_CAN_ID = 96


class MotorControlInterface(Node):
    def __init__(self):
        super().__init__("motor_control")
        
        self.motor_movement_timer = self.create_timer(
            callback=self.control_motors
        )

        self.teleop_subscriber = self.create_subscription(
            msg_type=XboxVelocityInput,
            topic="controller_move_state",
            callback=self.read_control_states,
            qos_profile=VOLATILE_QOS,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.vesc = VescCommandInterface(
            uart_port=UART_PORT,
            baud_rate=BAUD_RATE,
            secondary_can_id=LEFT_CAN_ID,
        )
        self.vesc.connect()

    def read_control_states(self, data):
        pass

    def control_motors(self):
        pass


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
