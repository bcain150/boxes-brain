from threading import RLock

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy import logging as ros_logger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from boxes_interfaces.msg import XboxInput
from boxes_interfaces.srv import SpeedTrim
from boxes_utils import VOLATILE_QOS, RELIABLE_QOS, format_error_message, locked

from speed.vesc_uart import VescCommandInterface, MotorStatusMsg

# VESC connection values
UART_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200
LEFT_CAN_ID = 96

MOTOR_STATE_READ_S = 0.04

# motor smoothing constants
# TODO: maybe should be ros params?
# DEFAULT_DUTY_SCALING = 0.10
MIN_DUTY_AGGREGATE = 0.05
MIN_RPM_AGGREGATE = 100
EMA_ALPHA = 0.05     

DUTY_SCALES = [0.10, 0.15, 0.30, 0.50, 0.75, 1.0]
TRIM_OFFSETS = []

class MotorControlInterface(Node):
    def __init__(self):
        super().__init__("motor_control")

        motor_control_group = MutuallyExclusiveCallbackGroup()
        motor_read_group = MutuallyExclusiveCallbackGroup()
        trim_group = ReentrantCallbackGroup()

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

        self.speed_trim_service = self.create_service(
            srv_type=SpeedTrim,
            srv_name="trim_speed",
            qos_profile=RELIABLE_QOS,
            callback=self._trim_and_scale,
            callback_group=trim_group,
        )

        self.vesc = VescCommandInterface(
            uart_port=UART_PORT,
            baud_rate=BAUD_RATE,
            secondary_can_id=LEFT_CAN_ID,
        )
        self.vesc.connect()

        self._serial_lock = RLock()

        # exponential moving averages for mirroring rpms (some motors have different duty-rpms)
        self.left_rpm_per_duty = None
        self.right_rpm_per_duty = None

        # trims calculated based off of averages
        self.rpm_trim_left = 1
        self.rpm_trim_right = 1
        self._trim_idx = 0

        # duty scaling
        self._duty_scaling_idx = 0  

    @property
    def right_scaling(self):
        return DUTY_SCALES[self._duty_scaling_idx]*self.rpm_trim_right
    
    @property
    def left_scaling(self):
        return DUTY_SCALES[self._duty_scaling_idx]*self.rpm_trim_left

    def read_motors(self):
        motor_fields = ["temp_motor", "temp_fet", "rpm", "fault_code", "duty_now"]
        with locked(self._serial_lock):
            try:
                self.get_logger().info("Getting left and right motor status...")
                right_motor = self.vesc.get_status(*motor_fields, to_can=True)
                left_motor = self.vesc.get_status(*motor_fields)
            except Exception as e:
                self.get_logger().warning(f"Error occured while getting motor status -> \n{format_error_message(e)}")
                return
            self.get_logger().info(f"-- RIGHT MOTOR STATUS --\n{right_motor}")
            self.get_logger().info(f"-- LEFT MOTOR STATUS --\n{left_motor}")
            self.get_logger().info(f"CURRENT SPEED SETTING: {DUTY_SCALES[self._duty_scaling_idx]}")

        self.right_rpm_per_duty = self._characterize_motor(right_motor, self.right_rpm_per_duty)
        self.left_rpm_per_duty = self._characterize_motor(left_motor, self.left_rpm_per_duty)

        if self.right_rpm_per_duty and self.left_rpm_per_duty:
            if self.left_rpm_per_duty < self.right_rpm_per_duty:
                self.rpm_trim_left = 1
                self.rpm_trim_right = self.left_rpm_per_duty / self.right_rpm_per_duty
            else:
                self.rpm_trim_left = self.right_rpm_per_duty / self.left_rpm_per_duty
                self.rpm_trim_right = 1

    def control_motors(self, data):
        self.get_logger().debug("Xbox Button Input Received...")
        left_motor_duty = data.left_stick_y
        right_motor_duty = data.right_stick_y
        is_connected = data.connected
        errored = False

        with locked(self._serial_lock):
            if is_connected:
                try:
                    self.vesc.set_duty(right_motor_duty*self.right_scaling, to_can=True)
                except Exception as e:
                    self.get_logger().error(f"Error setting right motor duty! ->\n{format_error_message(e)}")
                    errored = True
                try:
                    self.vesc.set_duty(left_motor_duty*self.left_scaling)
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
            self.get_logger().debug("Successfully set motor control...")

    def _trim_and_scale(self, request, response):
        self.get_logger().info(f"Received request to adjust speed scaling or motor trim! ->\n{request}")
        speed_increment = request.speed_increment
        left_trim_increment = request.left_trim_increment
        right_trim_increment = request.right_trim_increment
        response.success = True
        if speed_increment:
            # if we have speed input, then increment the scaling index but pin it
            # betwen 0 and len of DUTY_SCALE list so there's no wrapping
            new_index = self._duty_scaling_idx + speed_increment
            self._duty_scaling_idx = max(0, min(len(DUTY_SCALES)-1, new_index))
            if 0 > new_index:
                update_string = f"Already at minimum speed scaling ({DUTY_SCALES[0]*100}%)"
            elif len(DUTY_SCALES)-1 < new_index:
                update_string = f"Alreadying at maxium speed scaling ({DUTY_SCALES[-1]*100}%)"
            else:
                update_string = f"Set speed scaling to {DUTY_SCALES[self._duty_scaling_idx]*100}%"
        elif left_trim_increment:
            update_string = "not implemented"
        elif right_trim_increment:
            update_string = "not_implemented"
        else:
            update_string = "No speed scaling or trim input in request (all fields 0)!"
            response.success = False

        response.update_string = update_string
        self.get_logger().info(update_string)

        return response


    def _characterize_motor(self, motor_status: MotorStatusMsg, current_avg: float):
        """We want to characterize a motor by taking a rolling average of it's duty cycle to rpm ratio.
        This is important because our motors have different internal resistance. This causes them to spin at
        different RPMs when at the same duty cycle."""

        duty = abs(motor_status.duty_now)
        rpm = abs(motor_status.rpm)

        # if noisy just return it back
        if duty < MIN_DUTY_AGGREGATE or rpm < MIN_RPM_AGGREGATE:
            return current_avg
        instant_ratio = rpm/duty
        if current_avg is None:
            return instant_ratio
        return EMA_ALPHA * instant_ratio + (1 - EMA_ALPHA)*current_avg

def main(args=None):
    print("Starting Motor Control Node...")
    rclpy.init(args=args)
    motor_control = MotorControlInterface()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(motor_control)
    motor_control.get_logger().info("Motor Control Node Intialized.")

    try:
        executor.spin()
    except KeyboardInterrupt:
        ros_logger.get_logger("executor_logger").warning(
            "Keyboard Interrupt - Shutting down Motor Control"
        )
    finally:
        executor.shutdown()   # stops spinning, joins internal threads
        motor_control.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
