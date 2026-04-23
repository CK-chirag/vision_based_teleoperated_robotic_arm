#!/usr/bin/env python3
"""
Description:
    Subscribes to /joint_states, converts radians to degrees,
    and sends joint angles to ESP32 via serial (PCA9685 -> Servos)
    Only sends data when joint angles change more than 0.5 degrees.
--------
Subscribing Topics:
    /joint_states - sensor_msgs/msg/JointState
--------
Author: Chirag Khanna
Date: April 22, 2026
"""

import math
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SerialReader(Node):
    def __init__(self):
        super().__init__("serial_reader")

        # Parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        port     = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value

        # Map URDF joint name -> PCA9685 channel
        # CH0-CH3 -> MG966R, CH4-CH5 -> SG90
        self.joint_channel_map = {
            "link1_to_link2":        0,  # CH0 - MG966R (base)
            "link2_to_link3":        1,  # CH1 - MG966R
            "link3_to_link4":        2,  # CH2 - MG966R
            "link4_to_link5":        3,  # CH3 - MG966R
            "link5_to_link6":        4,  # CH4 - SG90
            "link6_to_link6_flange": 5,  # CH5 - SG90
        }

        # Track last sent angles
        self.last_angles = None

        # Serial connection
        try:
            self.esp32 = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to ESP32 on {port} at {baudrate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open port {port}: {e}")
            raise SystemExit(1)

        # Subscriber
        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

        self.get_logger().info("Waiting for /joint_states...")

    def angles_changed(self, new_angles):
        """Return True only if any angle changed more than 0.5 degrees."""
        if self.last_angles is None:
            return True
        for a, b in zip(new_angles, self.last_angles):
            if abs(a - b) > 0.5:
                return True
        return False

    def joint_state_callback(self, msg: JointState):
        try:
            angles = [90.0] * 6  # default all to center

            for i, name in enumerate(msg.name):
                if name in self.joint_channel_map:
                    ch = self.joint_channel_map[name]
                    deg = math.degrees(msg.position[i]) + 90.0
                    deg = max(0.0, min(180.0, deg))
                    angles[ch] = round(deg, 2)

            # Only send if angles changed more than 0.5 deg
            if not self.angles_changed(angles):
                return

            self.last_angles = angles

            # Format: J:90.0,90.0,90.0,90.0,90.0,90.0
            data = "J:" + ",".join(str(a) for a in angles) + "\n"
            self.esp32.write(data.encode())
            self.get_logger().info(f"Sent: {data.strip()}")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def destroy_node(self):
        if hasattr(self, 'esp32') and self.esp32.is_open:
            self.esp32.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()