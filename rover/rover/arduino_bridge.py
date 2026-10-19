#!/usr/bin/env python3

import math
import threading

import rclpy
import serial

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class ArduinoBridge(Node):

    def __init__(self):

        super().__init__('arduino_bridge')

        # ============================================================
        # SERIAL CONFIGURATION
        # ============================================================

        # Arduino:
        # Serial.begin(115200);
        self.declare_parameter(
            'port',
            '/dev/ttyACM0'
        )

        self.declare_parameter(
            'baudrate',
            115200
        )

        port = self.get_parameter(
            'port'
        ).value

        baudrate = self.get_parameter(
            'baudrate'
        ).value

        # ============================================================
        # ANGULAR VELOCITY REMAPPING
        # ============================================================

        self.declare_parameter(
            'ang_in_min',
            0.01
        )

        self.declare_parameter(
            'ang_in_max',
            0.20
        )

        self.declare_parameter(
            'ang_out_min',
            0.15
        )

        self.declare_parameter(
            'ang_out_max',
            0.20
        )

        self.declare_parameter(
            'zero_eps',
            0.001
        )

        self.declare_parameter(
            'remap_while_moving',
            False
        )

        self.declare_parameter(
            'in_place_v',
            0.01
        )

        self.in_min = self.get_parameter(
            'ang_in_min'
        ).value

        self.in_max = self.get_parameter(
            'ang_in_max'
        ).value

        self.out_min = self.get_parameter(
            'ang_out_min'
        ).value

        self.out_max = self.get_parameter(
            'ang_out_max'
        ).value

        self.zero_eps = self.get_parameter(
            'zero_eps'
        ).value

        self.remap_while_moving = self.get_parameter(
            'remap_while_moving'
        ).value

        self.in_place_v = self.get_parameter(
            'in_place_v'
        ).value

        # ============================================================
        # CONNECT TO ARDUINO
        # ============================================================

        try:

            self.serial_port = serial.Serial(
                port,
                baudrate,
                timeout=0.1,
                write_timeout=0.2
            )

            # Opening the Mega's USB serial can reset it.
            # Give it time to boot.
            import time
            time.sleep(2.0)

            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()

            self.get_logger().info(
                f'Connected to Arduino on {port} '
                f'at {baudrate} baud.'
            )

        except serial.SerialException as e:

            self.get_logger().error(
                f'Failed to connect to Arduino: {e}'
            )

            raise SystemExit

        # ============================================================
        # ROS SUBSCRIBER
        # ============================================================

        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10
        )

        # ============================================================
        # RAW SERIAL PUBLISHER
        # ============================================================

        # Parser node can subscribe to this.
        self.raw_pub = self.create_publisher(
            String,
            'raw_serial',
            50
        )

        # ============================================================
        # SERIAL READ THREAD
        # ============================================================

        self.read_thread = threading.Thread(
            target=self.read_serial_loop,
            daemon=True
        )

        self.read_thread.start()

        self.get_logger().info(
            'Arduino bridge ready.'
        )

    # ================================================================
    # ANGULAR VELOCITY REMAPPING
    # ================================================================

    def remap_angular(self, w, v):

        """
        Map angular velocity.

        When rotating in place:
            input  [ang_in_min, ang_in_max]
                ->
            output [ang_out_min, ang_out_max]

        When driving forward/backward:
            preserve the commanded curvature and only clamp
            the maximum angular velocity.
        """

        # Very small angular velocity = zero
        if abs(w) < self.zero_eps:
            return 0.0

        # ------------------------------------------------------------
        # Driving
        # ------------------------------------------------------------

        if (
            not self.remap_while_moving
            and abs(v) > self.in_place_v
        ):

            return math.copysign(
                min(abs(w), self.out_max),
                w
            )

        # ------------------------------------------------------------
        # In-place rotation
        # ------------------------------------------------------------

        mag = min(
            max(abs(w), self.in_min),
            self.in_max
        )

        # Avoid divide-by-zero if parameters are accidentally equal
        if self.in_max <= self.in_min:

            return math.copysign(
                self.out_max,
                w
            )

        t = (
            (mag - self.in_min)
            /
            (self.in_max - self.in_min)
        )

        out = (
            self.out_min
            +
            t * (self.out_max - self.out_min)
        )

        return math.copysign(
            out,
            w
        )

    # ================================================================
    # CMD_VEL CALLBACK
    # ================================================================

    def cmd_callback(self, msg):

        v = float(msg.linear.x)

        w = float(msg.angular.z)

        # Apply angular remapping
        w = self.remap_angular(
            w,
            v
        )

        # ============================================================
        # NEW ARDUINO PROTOCOL
        # ============================================================
        #
        # Arduino expects:
        #
        #     V <linear_velocity> <angular_velocity>
        #
        # Example:
        #
        #     V 0.1000 0.0000
        #
        # Old bridge sent:
        #
        #     0.1000,0.0000
        #
        # which the new Arduino does NOT understand.
        # ============================================================

        cmd_str = (
            f'V {v:.4f} {w:.4f}\n'
        )

        if self.serial_port.is_open:

            try:

                self.serial_port.write(
                    cmd_str.encode('ascii')
                )

                self.serial_port.flush()

            except serial.SerialException as e:

                self.get_logger().error(
                    f'Serial write error: {e}'
                )

    # ================================================================
    # SERIAL READER
    # ================================================================

    def read_serial_loop(self):

        """
        Reads everything coming from Arduino.

        Important:
        The new Arduino telemetry is:

        D,timestamp,LR,RR,yaw,pitch,roll,
        gyroX,gyroY,gyroZ,accelX,accelY,accelZ

        Front encoder values RF/LF are no longer present.
        """

        while (
            rclpy.ok()
            and self.serial_port.is_open
        ):

            try:

                if self.serial_port.in_waiting > 0:

                    line = (
                        self.serial_port
                        .readline()
                        .decode(
                            'utf-8',
                            errors='ignore'
                        )
                        .strip()
                    )

                    if not line:
                        continue

                    raw_msg = String()
                    raw_msg.data = line

                    self.raw_pub.publish(
                        raw_msg
                    )

            except serial.SerialException as e:

                self.get_logger().warn(
                    f'Serial read error: {e}'
                )

                break

            except Exception as e:

                self.get_logger().warn(
                    f'Serial processing error: {e}'
                )

    # ================================================================
    # SAFE SHUTDOWN
    # ================================================================

    def stop_rover(self):

        """
        Send zero velocity before closing serial.

        Arduino receives:
            V 0.0000 0.0000

        and its firmware will brake the rear motors.
        """

        if (
            self.serial_port is not None
            and self.serial_port.is_open
        ):

            try:

                self.serial_port.write(
                    b'V 0.0000 0.0000\n'
                )

                self.serial_port.flush()

                self.get_logger().info(
                    'Stop command sent to Arduino.'
                )

            except Exception as e:

                self.get_logger().warn(
                    f'Failed to send stop command: {e}'
                )

    # ================================================================
    # DESTROY
    # ================================================================

    def destroy_node(self):

        # Stop rover first
        self.stop_rover()

        # Give serial transmission a moment
        import time
        time.sleep(0.1)

        # Close serial
        if (
            self.serial_port is not None
            and self.serial_port.is_open
        ):

            self.serial_port.close()

        super().destroy_node()


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = ArduinoBridge()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            'Keyboard interrupt received.'
        )

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
