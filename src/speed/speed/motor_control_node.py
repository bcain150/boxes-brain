from threading import RLock

import rclpy
from rclpy import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy import logging as ros_logger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from boxes_interfaces.msg import XboxInput
from boxes_utils import VOLATILE_QOS, format_error_message

from speed.vesc_uart import VescCommandInterface

UART_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200
LEFT_CAN_ID = 96

MOTOR_STATE_READ_S = 0.5

class MotorControlInterface(Node):
    def __init__(self):
        super().__init__("motor_control")

        motor_control_group = MutuallyExclusiveCallbackGroup()
        motor_read_group = MutuallyExclusiveCallbackGroup()

        self.motor_state_timer = self.create_timer(
            timer_period_sec=MOTOR_STATE_READ_S,
            callback=self.read_motors,
            callback_group=motor_read_group
        )

        self.teleop_subscriber = self.create_subscription(
            msg_type=XboxInput,
            topic="controller_move_state",
            callback=self.control_motors,
            qos_profile=VOLATILE_QOS,
            callback_group=motor_control_group,
        )

        self.vesc = VescCommandInterface(
            uart_port=UART_PORT,
            baud_rate=BAUD_RATE,
            secondary_can_id=LEFT_CAN_ID,
        )
        self.vesc.connect()

        self._serial_lock = RLock()

    def read_motors(self):
        motor_fields = ["temp_motor", "temp_fet", "rpm", "fault_code", "duty_now"]
        with self._serial_lock:
            try:
                self.get_logger().info("Getting left and right motor status...")
                right_motor_status = self.vesc.get_status(*motor_fields)
                left_motor_status = self.vesc.get_status(*motor_fields, to_can=True)
            except Exception as e:
                self.get_logger().warning(f"Error occured while getting motor status -> \n{format_error_message(e)}")
            self.get_logger().debug(f"-- RIGHT MOTOR STATUS --\n{right_motor_status}")
            self.get_logger().debug(f"-- LEFT MOTOR STATUS --\n{left_motor_status}")

        # TODO: do something with these statuses (probably smothing and normalization)

    def control_motors(self, data):
        self.get_logger().info("Xbox Button Input Received...")
        left_motor_duty = data.left_stick_y
        right_motor_duty = data.right_stick_y
        is_connected = data.connected
        errored = False
        with self._serial_lock:
            if is_connected:
                try:
                    self.vesc.set_duty(right_motor_duty)
                except Exception as e:
                    self.get_logger().error(f"Error setting right motor duty! ->\n{format_error_message(e)}")
                    errored = True
                try:
                    self.vesc.set_duty(left_motor_duty, to_can=True)
                    errored = True
                except Exception as e:
                    self.get_logger().error(f"Error setting left motor duty! ->\n{format_error_message(e)}")
            else:
                try:
                    self.get_logger().warning("Controller disconnected - stopping motors...")
                    self.vesc.stop_all()
                except Exception as e:
                    self.get_logger().error(f"Error stopping motors! ->\n{format_error_message(e)}")
        
        if not errored:
            self.get_logger().info("Successfully set motor control...")

def main(args=None):
    print("Starting Motor Control Node...")
    rclpy.init(args=args)
    motor_control = MotorControlInterface()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(motor_control)
    motor_control.get_logger().info("Motor Control Node Intialized.")

    try:
        executor.spin()
    except KeyboardInterrupt:
        ros_logger.get_logger("executor_logger").warning(
            "Keyboard Interrupt - Shutting down Motor Control"
        )
    finally:
        motor_control.destroy_node()
        executor.shutdown()   # stops spinning, joins internal threads
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
