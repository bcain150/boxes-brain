#! /usr/bin/python3
import time
from typing import Optional

from evdev import (
    InputDevice,
    InputEvent,
    list_devices,
    ecodes
)

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy import logging as ros_logger

from boxes_interfaces.msg import XboxVelocityInput, XboxOutput
from boxes_utils import VOLATILE_QOS, format_error_message

from teleop.controller import (
    ControllerState,
    Button,
    VENDOR_ID, 
    PRODUCT_IDS
)

INPUT_PUBLISHING_RATE_S = 0.02

class XboxControllerInterface(Node):
    def __init__(self):
        super().__init__("xbox_control_interface")

        self.controller_read_timer = self.create_timer(0, self.read_controller)
        self.teleop_publishing_timer = self.create_timer(INPUT_PUBLISHING_RATE_S, self.send_teleop)

        self.teleop_publisher = self.create_publisher(
            msg_type=XboxVelocityInput,
            topic="controller_input",
            qos_profile=VOLATILE_QOS,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # allows other nodes to provide feedback
        self.controller_feedback = self.create_subscription(
            msg_type=XboxOutput,
            topic="controller_feedback",
            callback=self.write_controller,
            qos_profile=VOLATILE_QOS,
            callback=MutuallyExclusiveCallbackGroup()
        )
        
        # a place to store the instance of the connected controller
        self.controller: Optional[InputDevice] = None
        self.controller_state: Optional[ControllerState] = None

    
    def _connect_to_controller(self):
        while self.controller is None:
            for path in list_devices():
                device = InputDevice(path)
                if device.info.vendor == VENDOR_ID and device.info.product in PRODUCT_IDS:
                    self.get_logger().info(f"Device {device.name} was discovered!")
                    self.controller = device
                    self.controller_state = ControllerState(device=device, node_logger=self.get_logger())
                    break
            self.get_logger().warning("No device found!")
            time.sleep(3)
        self.get_logger().info("Controller Connected!")
            

    def read_controller(self):
        """ROS timer loop which constantly reads events coming from the controller. If the controller disconnects,
        it moves to a disconnected state and will constantly attempt to reconnect"""

        if not self.controller:
            # TODO: publish a state message that we are in the disconnected state
            # spin inside this loop until a controller is disconnected
            self._connect_to_controller()

        try:
            for event in self.controller.read_loop():
                if event.type == ecodes.EV_SYN:
                    continue
                self.get_logger().debug(f"Received event -> {event}")
                self.controller_state.update(event)
        except OSError:
            # clear controller instance since it has been disconnected
            self.controller = None
            self.get_logger().error("Controller Disconnected!")
        except Exception as e:
            self.get_logger().error(f"An unhandled exception occured in the controller read loop! ->\n{format_error_message(e)}")

    def write_controller(self):
        pass
    
    def send_teleop(self):
        # TODO read from state and publish teleop here
        

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