# custom_ekf.py
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
    Small planar EKF for the rover.

    State:
        x, y, yaw, gyro_z_bias

    Inputs:
        /wheel_odom : x/y position only
        /imu/data   : angular_velocity.z only

    Intentionally NOT fused:
        - wheel yaw
        - wheel angular velocity
        - IMU orientation
        - IMU linear acceleration

    This is designed for a skid-steer rover where encoder yaw can be
    unreliable during turns and the IMU's absolute heading should not
    fight the gyro/SLAM system.
    """

    def __init__(self):
        super().__init__('custom_ekf')

        self.declare_parameter('frequency', 50.0)
        self.declare_parameter('imu_timeout', 0.25)
        self.declare_parameter('wheel_position_std', 0.05)
        self.declare_parameter('gyro_z_std', 0.01)
        self.declare_parameter('gyro_bias_std', 0.0001)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.frequency = float(self.get_parameter('frequency').value)
        self.imu_timeout = float(self.get_parameter('imu_timeout').value)
        wheel_std = float(self.get_parameter('wheel_position_std').value)
        gyro_std = float(self.get_parameter('gyro_z_std').value)
        bias_std = float(self.get_parameter('gyro_bias_std').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        # State: [x, y, yaw, gyro_z_bias]
        self.x = np.array([
            0.0,
            0.0,
            float(self.get_parameter('initial_yaw').value),
            0.0,
        ], dtype=float)

        # Initial covariance.
        self.P = np.diag([
            wheel_std ** 2,
            wheel_std ** 2,
            math.radians(5.0) ** 2,
            bias_std ** 2,
        ])

        # Measurement covariance for wheel x/y position.
        self.R_wheel = np.diag([wheel_std ** 2, wheel_std ** 2])

        # Process noise spectral densities.
        self.q_x = (0.02 ** 2)       # m^2/s
        self.q_y = (0.02 ** 2)       # m^2/s
        self.q_yaw = gyro_std ** 2   # rad^2/s
        self.q_bias = bias_std ** 2  # (rad/s)^2/s

        self.latest_gz = 0.0
        self.last_imu_time: Optional[float] = None
        self.last_predict_time: Optional[float] = None
        self.initialized_from_wheel = False

        self.last_pub_x = None
        self.last_pub_y = None
        self.last_pub_time: Optional[float] = None

        self.pub_odom = self.create_publisher(
            Odometry, '/odometry/filtered', 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 20
        )
        self.create_subscription(
            Odometry, '/wheel_odom', self.wheel_callback, 20
        )

        period = 1.0 / max(self.frequency, 1.0)
        self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'Custom planar EKF started at {self.frequency:.1f} Hz'
        )
        self.get_logger().info(
            'Fusing wheel x/y + IMU gyro z only; '
            'IMU orientation and wheel yaw are ignored.'
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def imu_callback(self, msg: Imu):
        now = self.now_sec()
        self.latest_gz = float(msg.angular_velocity.z)
        self.last_imu_time = now

        if self.last_predict_time is None:
            self.last_predict_time = now

    def wheel_callback(self, msg: Odometry):
        # Wheel odometry contributes position x/y only.
        if not self.initialized_from_wheel:
            self.x[0] = float(msg.pose.pose.position.x)
            self.x[1] = float(msg.pose.pose.position.y)
            self.initialized_from_wheel = True
            self.get_logger().info(
                f'Initialized position from wheel_odom: '
                f'x={self.x[0]:.3f}, y={self.x[1]:.3f}'
            )
            return

        z = np.array([
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        ])

        # Measurement matrix H for x/y only.
        H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ])

        innovation = z - H @ self.x
        S = H @ self.P @ H.T + self.R_wheel

        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.get_logger().warning(
                'Singular innovation matrix; skipping wheel update.'
            )
            return

        self.x = self.x + K @ innovation

        # Joseph-form covariance update for numerical stability.
        I = np.eye(4)
        self.P = (
            (I - K @ H) @ self.P @ (I - K @ H).T
            + K @ self.R_wheel @ K.T
        )

        self.x[2] = normalize_angle(self.x[2])

    def predict(self, dt: float, gyro_z: float):
        if dt <= 0.0:
            return

        # State propagation:
        # x/y are corrected by wheel odometry measurements.
        # Yaw is propagated only from gyro z.
        gyro_corrected = gyro_z - self.x[3]
        self.x[2] = normalize_angle(
            self.x[2] + gyro_corrected * dt
        )

        # Jacobian F.
        F = np.eye(4)
        F[2, 3] = -dt

        # Continuous-noise approximation.
        Q = np.diag([
            self.q_x * dt,
            self.q_y * dt,
            self.q_yaw * dt,
            self.q_bias * dt,
        ])

        self.P = F @ self.P @ F.T + Q

        # Enforce symmetry to limit numerical asymmetry.
        self.P = 0.5 * (self.P + self.P.T)

    def timer_callback(self):
        now = self.now_sec()

        if not self.initialized_from_wheel:
            return

        if self.last_predict_time is None:
            self.last_predict_time = now

        dt = now - self.last_predict_time
        self.last_predict_time = now

        # Never integrate stale gyro data indefinitely.
        if (
            self.last_imu_time is None
            or (now - self.last_imu_time) > self.imu_timeout
        ):
            gyro_z = 0.0
        else:
            gyro_z = self.latest_gz

        self.predict(dt, gyro_z)
        self.publish(now, gyro_z)

    def publish(self, now: float, gyro_z: float):
        qx, qy, qz, qw = yaw_to_quaternion(self.x[2])

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = float(self.x[0])
        msg.pose.pose.position.y = float(self.x[1])
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Pose covariance: x, y, yaw only.
        msg.pose.covariance[0] = max(float(self.P[0, 0]), 1e-9)
        msg.pose.covariance[7] = max(float(self.P[1, 1]), 1e-9)
        msg.pose.covariance[35] = max(float(self.P[2, 2]), 1e-9)

        # Estimate planar linear velocity from filtered position changes.
        vx = 0.0
        vy = 0.0
        if self.last_pub_time is not None:
            dpub = now - self.last_pub_time
            if dpub > 1e-6:
                vx = (self.x[0] - self.last_pub_x) / dpub
                vy = (self.x[1] - self.last_pub_y) / dpub

        self.last_pub_x = float(self.x[0])
        self.last_pub_y = float(self.x[1])
        self.last_pub_time = now

        msg.twist.twist.linear.x = float(vx)
        msg.twist.twist.linear.y = float(vy)
        msg.twist.twist.linear.z = 0.0

        gyro_corrected = gyro_z - self.x[3]
        msg.twist.twist.angular.z = float(gyro_corrected)

        msg.twist.covariance[0] = 0.1 ** 2
        msg.twist.covariance[7] = 0.1 ** 2
        msg.twist.covariance[35] = max(self.q_yaw, 1e-6)

        self.pub_odom.publish(msg)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = float(self.x[0])
            tf.transform.translation.y = float(self.x[1])
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = CustomEKF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
