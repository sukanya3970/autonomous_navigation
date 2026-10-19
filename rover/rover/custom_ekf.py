#!/usr/bin/env python3

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class CustomEKF(Node):
    """
    Planar EKF for the skid-steer rover.

    State:
        x, y, yaw, gyro_z_bias

    Current measurements:
        /wheel_odom : x/y position
        /imu/data   : angular_velocity.z

    Intentionally NOT fused:
        - wheel yaw
        - wheel angular velocity
        - IMU absolute orientation
        - IMU linear acceleration

    Bias handling:
        A stationary zero-rate update is applied when both:
          1) wheel-derived linear speed is very small
          2) measured gyro-Z is very small

        The stationary gyro measurement is used as:

            gyro_z ~= gyro_z_bias

        This makes the gyro bias state observable while avoiding
        bias correction during a real in-place rotation.

    Important:
        The current /wheel_odom message does not contain independent
        left/right wheel velocity or displacement. Therefore wheel
        diameter and track width are not used directly in this EKF yet.

        Those parameters should be used in the encoder/odometry source
        when we move to the next version with direct wheel-motion input.
    """

    def __init__(self):
        super().__init__("custom_ekf")

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter("frequency", 50.0)
        self.declare_parameter("imu_timeout", 0.25)

        # Wheel X/Y measurement uncertainty.
        self.declare_parameter(
            "wheel_position_std",
            0.05
        )

        # Measured from the 60-second stationary IMU test:
        #
        # Mean: -0.000183333 rad/s
        # Std :  0.001337537 rad/s
        self.declare_parameter(
            "gyro_z_std",
            0.001337537
        )

        # Gyro bias random walk.
        self.declare_parameter(
            "gyro_bias_std",
            0.0001
        )

        # Initial gyro bias from stationary test.
        self.declare_parameter(
            "initial_gyro_bias",
            -0.000183333
        )

        self.declare_parameter(
            "initial_yaw",
            0.0
        )

        self.declare_parameter(
            "publish_tf",
            True
        )

        self.declare_parameter(
            "odom_frame",
            "odom"
        )

        self.declare_parameter(
            "base_frame",
            "base_link"
        )

        # ------------------------------------------------------------
        # Stationary detector
        # ------------------------------------------------------------

        # Wheel-derived linear speed below this value is considered
        # stationary.
        self.declare_parameter(
            "stationary_speed_threshold",
            0.01
        )

        # Gyro-Z must also be small before performing the stationary
        # bias update. This prevents the filter from trying to remove
        # a real in-place rotation.
        self.declare_parameter(
            "stationary_gyro_threshold",
            0.01
        )

        # Do not perform bias correction every EKF cycle.
        self.declare_parameter(
            "bias_update_interval",
            0.20
        )

        # ============================================================
        # Read parameters
        # ============================================================

        self.frequency = float(
            self.get_parameter(
                "frequency"
            ).value
        )

        self.imu_timeout = float(
            self.get_parameter(
                "imu_timeout"
            ).value
        )

        wheel_std = float(
            self.get_parameter(
                "wheel_position_std"
            ).value
        )

        gyro_std = float(
            self.get_parameter(
                "gyro_z_std"
            ).value
        )

        bias_std = float(
            self.get_parameter(
                "gyro_bias_std"
            ).value
        )

        initial_yaw = float(
            self.get_parameter(
                "initial_yaw"
            ).value
        )

        initial_bias = float(
            self.get_parameter(
                "initial_gyro_bias"
            ).value
        )

        self.publish_tf = bool(
            self.get_parameter(
                "publish_tf"
            ).value
        )

        self.odom_frame = str(
            self.get_parameter(
                "odom_frame"
            ).value
        )

        self.base_frame = str(
            self.get_parameter(
                "base_frame"
            ).value
        )

        self.stationary_speed_threshold = float(
            self.get_parameter(
                "stationary_speed_threshold"
            ).value
        )

        self.stationary_gyro_threshold = float(
            self.get_parameter(
                "stationary_gyro_threshold"
            ).value
        )

        self.bias_update_interval = float(
            self.get_parameter(
                "bias_update_interval"
            ).value
        )

        # ============================================================
        # EKF state
        # ============================================================

        # State:
        #
        #   x[0] = x
        #   x[1] = y
        #   x[2] = yaw
        #   x[3] = gyro_z_bias
        #
        self.x = np.array(
            [
                0.0,
                0.0,
                initial_yaw,
                initial_bias,
            ],
            dtype=float
        )

        # ============================================================
        # Initial covariance
        # ============================================================

        self.P = np.diag(
            [
                wheel_std ** 2,
                wheel_std ** 2,

                # Initial yaw uncertainty = 5 degrees.
                math.radians(5.0) ** 2,

                # Initial gyro bias uncertainty.
                max(
                    bias_std ** 2,
                    1e-10
                ),
            ]
        )

        # ============================================================
        # Wheel position measurement covariance
        # ============================================================

        self.R_wheel = np.diag(
            [
                wheel_std ** 2,
                wheel_std ** 2,
            ]
        )

        # ============================================================
        # Process noise
        # ============================================================

        # Position process noise.
        self.q_x = 0.02 ** 2
        self.q_y = 0.02 ** 2

        # Gyro Z process noise.
        self.q_yaw = gyro_std ** 2

        # Gyro bias random walk.
        self.q_bias = bias_std ** 2

        # ============================================================
        # Stationary gyro bias measurement covariance
        # ============================================================

        self.R_bias = gyro_std ** 2

        # ============================================================
        # Sensor state
        # ============================================================

        self.latest_gz = 0.0

        self.last_imu_time: Optional[float] = None

        self.last_predict_time: Optional[float] = None

        self.initialized_from_wheel = False

        # ============================================================
        # Wheel history
        # ============================================================

        self.last_wheel_x: Optional[float] = None
        self.last_wheel_y: Optional[float] = None
        self.last_wheel_time: Optional[float] = None

        # Estimated translational speed from wheel odometry.
        #
        # This is currently used only for stationary detection.
        self.wheel_speed = 0.0

        # ============================================================
        # Bias update timing
        # ============================================================

        self.last_bias_update_time: Optional[float] = None

        # ============================================================
        # Published velocity history
        # ============================================================

        self.last_pub_x: Optional[float] = None
        self.last_pub_y: Optional[float] = None
        self.last_pub_time: Optional[float] = None

        # ============================================================
        # ROS publishers
        # ============================================================

        self.pub_odom = self.create_publisher(
            Odometry,
            "/odometry/filtered",
            10
        )

        self.tf_broadcaster = TransformBroadcaster(
            self
        )

        # ============================================================
        # ROS subscribers
        # ============================================================

        self.create_subscription(
            Imu,
            "/imu/data",
            self.imu_callback,
            20
        )

        self.create_subscription(
            Odometry,
            "/wheel_odom",
            self.wheel_callback,
            20
        )

        # ============================================================
        # EKF timer
        # ============================================================

        period = 1.0 / max(
            self.frequency,
            1.0
        )

        self.create_timer(
            period,
            self.timer_callback
        )

        # ============================================================
        # Startup logging
        # ============================================================

        self.get_logger().info(
            f"Custom planar EKF started at "
            f"{self.frequency:.1f} Hz"
        )

        self.get_logger().info(
            "Fusing wheel x/y + IMU gyro Z only."
        )

        self.get_logger().info(
            "IMU orientation and wheel yaw are ignored."
        )

        self.get_logger().info(
            f"Initial gyro-Z bias: "
            f"{initial_bias:+.9f} rad/s"
        )

        self.get_logger().info(
            f"Gyro-Z noise std: "
            f"{gyro_std:.9f} rad/s"
        )

        self.get_logger().info(
            "Stationary detector: "
            f"speed < "
            f"{self.stationary_speed_threshold:.3f} m/s, "
            f"|gyro_z| < "
            f"{self.stationary_gyro_threshold:.4f} rad/s"
        )

    # ================================================================
    # Time helper
    # ================================================================

    def now_sec(self) -> float:

        return (
            self.get_clock()
            .now()
            .nanoseconds
            * 1e-9
        )

    # ================================================================
    # IMU callback
    # ================================================================

    def imu_callback(
        self,
        msg: Imu
    ):

        now = self.now_sec()

        self.latest_gz = float(
            msg.angular_velocity.z
        )

        self.last_imu_time = now

        if self.last_predict_time is None:

            self.last_predict_time = now

    # ================================================================
    # Wheel callback
    # ================================================================

    def wheel_callback(
        self,
        msg: Odometry
    ):

        """
        Wheel odometry contributes X/Y position only.

        Wheel yaw from the message is deliberately not fused.

        The change in wheel X/Y is also used to estimate whether the
        rover is stationary.
        """

        wheel_x = float(
            msg.pose.pose.position.x
        )

        wheel_y = float(
            msg.pose.pose.position.y
        )

        now = self.now_sec()

        # ------------------------------------------------------------
        # Initialize from first wheel position.
        # ------------------------------------------------------------

        if not self.initialized_from_wheel:

            self.x[0] = wheel_x
            self.x[1] = wheel_y

            self.last_wheel_x = wheel_x
            self.last_wheel_y = wheel_y
            self.last_wheel_time = now

            self.initialized_from_wheel = True

            self.get_logger().info(
                "Initialized position from wheel_odom: "
                f"x={self.x[0]:.3f}, "
                f"y={self.x[1]:.3f}"
            )

            return

        # ------------------------------------------------------------
        # Estimate wheel-derived translational speed.
        #
        # IMPORTANT:
        # This is NOT used as a yaw measurement.
        #
        # It is only used to decide whether the rover is stationary.
        # ------------------------------------------------------------

        if (
            self.last_wheel_x is not None
            and self.last_wheel_y is not None
            and self.last_wheel_time is not None
        ):

            dt = (
                now
                - self.last_wheel_time
            )

            if (
                dt > 1e-4
                and dt < 1.0
            ):

                dx = (
                    wheel_x
                    - self.last_wheel_x
                )

                dy = (
                    wheel_y
                    - self.last_wheel_y
                )

                distance = math.hypot(
                    dx,
                    dy
                )

                self.wheel_speed = (
                    distance / dt
                )

        self.last_wheel_x = wheel_x
        self.last_wheel_y = wheel_y
        self.last_wheel_time = now

        # ------------------------------------------------------------
        # Wheel position measurement update
        # ------------------------------------------------------------

        z = np.array(
            [
                wheel_x,
                wheel_y,
            ],
            dtype=float
        )

        # Measurement matrix:
        #
        # z = [x, y]
        #
        # Therefore:
        #
        # H = [1 0 0 0]
        #     [0 1 0 0]
        #
        H = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ],
            dtype=float
        )

        innovation = (
            z
            - H @ self.x
        )

        S = (
            H @ self.P @ H.T
            + self.R_wheel
        )

        try:

            K = (
                self.P
                @ H.T
                @ np.linalg.inv(S)
            )

        except np.linalg.LinAlgError:

            self.get_logger().warning(
                "Singular innovation matrix; "
                "skipping wheel update."
            )

            return

        # EKF state update.
        self.x = (
            self.x
            + K @ innovation
        )

        # ------------------------------------------------------------
        # Joseph-form covariance update.
        # ------------------------------------------------------------

        I = np.eye(4)

        self.P = (
            (I - K @ H)
            @ self.P
            @ (I - K @ H).T
            + K @ self.R_wheel @ K.T
        )

        self.P = 0.5 * (
            self.P
            + self.P.T
        )

        self.x[2] = normalize_angle(
            self.x[2]
        )

    # ================================================================
    # Stationary gyro bias update
    # ================================================================

    def stationary_bias_update(
        self,
        gyro_z: float,
        now: float
    ):

        """
        Zero-rate stationary update.

        During a true stationary period:

            true angular velocity ~= 0

        Therefore:

            measured gyro_z ~= gyro_z_bias

        We use that as a scalar EKF measurement of the bias state.

        The update is allowed only when:

            wheel speed is small
            AND
            gyro Z magnitude is small

        This prevents the bias estimator from trying to cancel
        a genuine in-place rover rotation.
        """

        # ------------------------------------------------------------
        # Limit bias correction rate
        # ------------------------------------------------------------

        if self.last_bias_update_time is not None:

            if (
                now
                - self.last_bias_update_time
                < self.bias_update_interval
            ):

                return

        # ------------------------------------------------------------
        # Stationary condition
        # ------------------------------------------------------------

        stationary = (
            self.wheel_speed
            <= self.stationary_speed_threshold
            and
            abs(gyro_z)
            <= self.stationary_gyro_threshold
        )

        if not stationary:

            return

        # ------------------------------------------------------------
        # Scalar measurement model:
        #
        # z = gyro_z
        #
        # h(x) = gyro_z_bias
        #
        # H = [0 0 0 1]
        # ------------------------------------------------------------

        H = np.array(
            [
                [
                    0.0,
                    0.0,
                    0.0,
                    1.0
                ]
            ],
            dtype=float
        )

        z = np.array(
            [
                gyro_z
            ],
            dtype=float
        )

        predicted_measurement = np.array(
            [
                self.x[3]
            ],
            dtype=float
        )

        innovation = (
            z
            - predicted_measurement
        )

        S = (
            H @ self.P @ H.T
            + self.R_bias
        )

        try:

            K = (
                self.P
                @ H.T
                @ np.linalg.inv(S)
            )

        except np.linalg.LinAlgError:

            self.get_logger().warning(
                "Singular bias innovation matrix; "
                "skipping stationary bias update."
            )

            return

        # ------------------------------------------------------------
        # State update
        # ------------------------------------------------------------

        self.x = (
            self.x
            + (
                K @ innovation
            ).reshape(-1)
        )

        # ------------------------------------------------------------
        # Joseph-form covariance update
        # ------------------------------------------------------------

        I = np.eye(4)

        R = np.array(
            [
                [self.R_bias]
            ],
            dtype=float
        )

        self.P = (
            (I - K @ H)
            @ self.P
            @ (I - K @ H).T
            + K @ R @ K.T
        )

        self.P = 0.5 * (
            self.P
            + self.P.T
        )

        self.x[2] = normalize_angle(
            self.x[2]
        )

        self.last_bias_update_time = now

    # ================================================================
    # Prediction
    # ================================================================

    def predict(
        self,
        dt: float,
        gyro_z: float
    ):

        if dt <= 0.0:

            return

        # Correct gyro using current estimated bias.

        gyro_corrected = (
            gyro_z
            - self.x[3]
        )

        # ------------------------------------------------------------
        # Yaw prediction
        # ------------------------------------------------------------

        self.x[2] = normalize_angle(
            self.x[2]
            + gyro_corrected * dt
        )

        # ------------------------------------------------------------
        # State transition Jacobian
        # ------------------------------------------------------------

        F = np.eye(4)

        # yaw(k+1) depends on bias(k)

        F[2, 3] = -dt

        # ------------------------------------------------------------
        # Process noise
        # ------------------------------------------------------------

        Q = np.diag(
            [
                self.q_x * dt,
                self.q_y * dt,
                self.q_yaw * dt,
                self.q_bias * dt,
            ]
        )

        # ------------------------------------------------------------
        # Covariance propagation
        # ------------------------------------------------------------

        self.P = (
            F @ self.P @ F.T
            + Q
        )

        # Force symmetry.

        self.P = 0.5 * (
            self.P
            + self.P.T
        )

    # ================================================================
    # EKF timer
    # ================================================================

    def timer_callback(self):

        now = self.now_sec()

        # No wheel position yet.

        if not self.initialized_from_wheel:

            return

        # First timer iteration.

        if self.last_predict_time is None:

            self.last_predict_time = now

        dt = (
            now
            - self.last_predict_time
        )

        self.last_predict_time = now

        if dt <= 0.0:

            return

        # Prevent an unusually large timer gap from causing a huge
        # single prediction step.

        if dt > 1.0:

            dt = 1.0

        # ------------------------------------------------------------
        # IMU timeout protection
        # ------------------------------------------------------------

        if (
            self.last_imu_time is None
            or
            (
                now
                - self.last_imu_time
                > self.imu_timeout
            )
        ):

            gyro_z = 0.0

        else:

            gyro_z = self.latest_gz

        # ------------------------------------------------------------
        # Stationary zero-rate bias update
        # ------------------------------------------------------------

        self.stationary_bias_update(
            gyro_z,
            now
        )

        # ------------------------------------------------------------
        # EKF prediction
        # ------------------------------------------------------------

        self.predict(
            dt,
            gyro_z
        )

        # ------------------------------------------------------------
        # Publish
        # ------------------------------------------------------------

        self.publish(
            now,
            gyro_z
        )

    # ================================================================
    # Publish filtered odometry and TF
    # ================================================================

    def publish(
        self,
        now: float,
        gyro_z: float
    ):

        qx, qy, qz, qw = (
            yaw_to_quaternion(
                self.x[2]
            )
        )

        msg = Odometry()

        # ------------------------------------------------------------
        # Header
        # ------------------------------------------------------------

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            self.odom_frame
        )

        msg.child_frame_id = (
            self.base_frame
        )

        # ------------------------------------------------------------
        # Position
        # ------------------------------------------------------------

        msg.pose.pose.position.x = float(
            self.x[0]
        )

        msg.pose.pose.position.y = float(
            self.x[1]
        )

        msg.pose.pose.position.z = 0.0

        # ------------------------------------------------------------
        # Orientation
        # ------------------------------------------------------------

        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # ------------------------------------------------------------
        # Pose covariance
        # ------------------------------------------------------------

        msg.pose.covariance[0] = max(
            float(self.P[0, 0]),
            1e-9
        )

        msg.pose.covariance[7] = max(
            float(self.P[1, 1]),
            1e-9
        )

        msg.pose.covariance[35] = max(
            float(self.P[2, 2]),
            1e-9
        )

        # ------------------------------------------------------------
        # Filtered linear velocity
        #
        # At the current stage this is derived from the filtered
        # position. Once the Arduino publishes independent wheel
        # velocity, this should be replaced by encoder-derived
        # velocity.
        # ------------------------------------------------------------

        vx = 0.0
        vy = 0.0

        if (
            self.last_pub_time is not None
            and self.last_pub_x is not None
            and self.last_pub_y is not None
        ):

            dpub = (
                now
                - self.last_pub_time
            )

            if dpub > 1e-6:

                vx = (
                    self.x[0]
                    - self.last_pub_x
                ) / dpub

                vy = (
                    self.x[1]
                    - self.last_pub_y
                ) / dpub

        self.last_pub_x = float(
            self.x[0]
        )

        self.last_pub_y = float(
            self.x[1]
        )

        self.last_pub_time = now

        msg.twist.twist.linear.x = float(
            vx
        )

        msg.twist.twist.linear.y = float(
            vy
        )

        msg.twist.twist.linear.z = 0.0

        # ------------------------------------------------------------
        # Angular velocity
        # ------------------------------------------------------------

        gyro_corrected = (
            gyro_z
            - self.x[3]
        )

        msg.twist.twist.angular.z = float(
            gyro_corrected
        )

        # ------------------------------------------------------------
        # Twist covariance
        # ------------------------------------------------------------

        msg.twist.covariance[0] = (
            0.1 ** 2
        )

        msg.twist.covariance[7] = (
            0.1 ** 2
        )

        msg.twist.covariance[35] = max(
            self.q_yaw,
            1e-9
        )

        self.pub_odom.publish(msg)

        # ============================================================
        # TF: odom -> base_link
        # ============================================================

        if self.publish_tf:

            tf = TransformStamped()

            tf.header.stamp = (
                msg.header.stamp
            )

            tf.header.frame_id = (
                self.odom_frame
            )

            tf.child_frame_id = (
                self.base_frame
            )

            tf.transform.translation.x = float(
                self.x[0]
            )

            tf.transform.translation.y = float(
                self.x[1]
            )

            tf.transform.translation.z = 0.0

            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(
                tf
            )


def main(args=None):

    rclpy.init(args=args)

    node = CustomEKF()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
