#! /usr/bin/python3
import time
from typing import Optional

from evdev import InputDevice, list_devices, ecodes

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy import logging as ros_logger

from boxes_interfaces.msg import XboxOutput, XboxInput
from boxes_utils import VOLATILE_QOS, format_error_message

from teleop.controller import ControllerState, Button, VENDOR_ID, PRODUCT_IDS

INPUT_PUBLISHING_RATE_S = 0.01


class XboxControllerInterface(Node):
    def __init__(self):
        super().__init__("xbox_control_interface")

        state_read_group = MutuallyExclusiveCallbackGroup()
        input_output_group = MutuallyExclusiveCallbackGroup()

        # controller read timer is a loop inside a loop so just immediately restart it
        self.controller_read_timer = self.create_timer(
            timer_period_sec=0,
            callback=self.read_controller,
            callback_group=state_read_group
        )
        self.teleop_publishing_timer = self.create_timer(
            timer_period_sec=INPUT_PUBLISHING_RATE_S,
            callback=self.send_teleop,
            callback_group=input_output_group
        )

        self.teleop_publisher = self.create_publisher(
            msg_type=XboxInput,
            topic="controller_move_state",
            qos_profile=VOLATILE_QOS
        )

        # allows other nodes to provide feedback
        self.controller_feedback = self.create_subscription(
            msg_type=XboxOutput,
            topic="controller_feedback",
            callback=self.write_controller,
            qos_profile=VOLATILE_QOS,
            callback_group=input_output_group,
        )

        # a place to store the instance of the connected controller
        self.controller: Optional[InputDevice] = None
        self.controller_state: Optional[ControllerState] = None
        self.first_read_success = False

    @property
    def connected(self):
        return self.controller is not None

    def _connect_to_controller(self):
        while self.controller is None:
            for path in list_devices():
                device = InputDevice(path)
                if (
                    device.info.vendor == VENDOR_ID
                    and device.info.product in PRODUCT_IDS
                ):
                    self.get_logger().info(f"Device {device.name} was discovered!")
                    self.controller = device
                    self.controller_state = ControllerState(
                        device=device, node_logger=self.get_logger()
                    )
                    break
            if self.connected:
                break
            self.get_logger().warning("No device found!")
            time.sleep(1)
        self.get_logger().info("Controller Connected!")

    def read_controller(self):
        """ROS timer loop which constantly reads events coming from the controller. If the controller disconnects,
        it moves to a disconnected state and will constantly attempt to reconnect"""

        if not self.connected:
            # spin inside this loop until a controller is disconnected
            self._connect_to_controller()

        try:
            for event in self.controller.read_loop():
                if event.type == ecodes.EV_SYN:
                    self.get_logger().debug("SYN RECEIVED")
                    continue
                self.get_logger().debug(f"Received event -> {event}")
                self.controller_state.update(event)
                self.first_read_success = True
        except OSError:
            # clear controller instance since it has been disconnected
            self.controller = None
            self.get_logger().error("Controller Disconnected!")
        except Exception as e:
            self.get_logger().error(
                f"An unhandled exception occured in the controller read loop! ->\n{format_error_message(e)}"
            )

    def write_controller(self, data):
        #TODO: implement rumbling
        pass

    def send_teleop(self):
        if not self.first_read_success:
            return
        try:
            input_state = XboxInput(
                connected=self.connected,
                left_stick_y=self.controller_state.get_normalized(Button.LEFT_STICK_Y),
                right_stick_y=self.controller_state.get_normalized(Button.RIGHT_STICK_Y)
            )
            self.get_logger().debug("Publishing Controller Input State...")
        except Exception as e:
            self.get_logger().error(f"Error Publishing Controller State! ->\n {format_error_message(e)}")

        self.teleop_publisher.publish(input_state)



def main(args=None):
    print("Starting Xbox Interface Node...")
    rclpy.init(args=args)
    xbox_node = XboxControllerInterface()

    # TODO: think about how many threads
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(xbox_node)
    xbox_node.get_logger().info("Xbox Control Interface Initialized.")

    try:
        executor.spin()
    except KeyboardInterrupt:
        ros_logger.get_logger("executor_logger").warning(
            "Keyboard Interrupt - Shutting down Xbox Control Interface!"
        )
    finally:
        xbox_node.destroy_node()
        executor.shutdown()   # stops spinning, joins internal threads
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
