import math
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


# ============================================================
# ARDUINO UINT32 TIMESTAMP
# ============================================================

UINT32_MAX = 0xFFFFFFFF
UINT32_HALF = 0x80000000


# ============================================================
# ROBOT CONSTANTS
# ============================================================

# These MUST match the Arduino firmware.

ENCODER_CPR = 4096.0
GEAR_RATIO = 3.6

# Arduino WHEEL_CIRC
WHEEL_CIRC = 0.32

# Arduino TRACK_WIDTH
TRACK_WIDTH = 0.55

# Encoder counts per wheel revolution
COUNTS_PER_WHEEL_REV = ENCODER_CPR * GEAR_RATIO


# ============================================================
# EULER -> QUATERNION
# ============================================================

def euler_to_quaternion(
    roll: float,
    pitch: float,
    yaw: float
):
    """
    Convert roll/pitch/yaw in radians to quaternion.
    """

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = (
        cr * cp * cy
        + sr * sp * sy
    )

    qx = (
        sr * cp * cy
        - cr * sp * sy
    )

    qy = (
        cr * sp * cy
        + sr * cp * sy
    )

    qz = (
        cr * cp * sy
        - sr * sp * cy
    )

    return qx, qy, qz, qw


# ============================================================
# SENSOR PARSER
# ============================================================

class SensorParser(Node):

    """
    Convert the NEW Arduino D telemetry into ROS messages.

    NEW ARDUINO FORMAT:

        D,<millis>,<LR>,<RR>,<yaw>,<pitch>,<roll>,
          <gyroX>,<gyroY>,<gyroZ>,
          <accelX>,<accelY>,<accelZ>

    Example:

        D,123456,10234,10456,12.30,0.10,-1.20,
          0.001,0.002,0.003,
          0.10,0.20,9.70

    Encoder layout:

        parts[0]  = D
        parts[1]  = Arduino millis
        parts[2]  = LR encoder
        parts[3]  = RR encoder
        parts[4]  = yaw
        parts[5]  = pitch
        parts[6]  = roll
        parts[7]  = gyro X
        parts[8]  = gyro Y
        parts[9]  = gyro Z
        parts[10] = accel X
        parts[11] = accel Y
        parts[12] = accel Z

    IMPORTANT:

        The Arduino encoder values are already logically signed.

        Do NOT apply encoder sign correction again here.
    """

    def __init__(self):

        super().__init__("sensor_parser")

        # ========================================================
        # PARAMETERS
        # ========================================================

        self.declare_parameter(
            "imu_frame",
            "imu_link"
        )

        self.declare_parameter(
            "odom_frame",
            "odom"
        )

        self.declare_parameter(
            "base_frame",
            "base_link"
        )

        # Gyro noise measured previously
        self.declare_parameter(
            "gyro_std",
            0.001337537
        )

        self.declare_parameter(
            "accel_std",
            0.10
        )

        self.imu_frame = str(
            self.get_parameter(
                "imu_frame"
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

        self.gyro_std = float(
            self.get_parameter(
                "gyro_std"
            ).value
        )

        self.accel_std = float(
            self.get_parameter(
                "accel_std"
            ).value
        )

        # ========================================================
        # ROS INTERFACES
        # ========================================================

        self.create_subscription(
            String,
            "/raw_serial",
            self.raw_callback,
            50
        )

        self.imu_pub = self.create_publisher(
            Imu,
            "/imu/data",
            20
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            "/wheel_odom",
            20
        )

        # ========================================================
        # ARDUINO TIMESTAMP STATE
        # ========================================================

        self.last_arduino_ms: Optional[int] = None

        self.timestamp_wraps = 0
        self.timestamp_backwards = 0

        self.timestamp_reset_active = False

        # ========================================================
        # ENCODER STATE
        # ========================================================

        self.previous_lr_count: Optional[int] = None
        self.previous_rr_count: Optional[int] = None

        self.previous_arduino_ms: Optional[int] = None

        # ========================================================
        # WHEEL ODOMETRY POSE
        # ========================================================

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        # ========================================================
        # COUNTERS
        # ========================================================

        self.imu_count = 0
        self.odom_count = 0

        # ========================================================
        # STARTUP INFO
        # ========================================================

        self.get_logger().info(
            "Sensor parser started — NEW D telemetry format."
        )

        self.get_logger().info(
            "Telemetry: D,timestamp,LR,RR,yaw,pitch,roll,"
            "gyroX,gyroY,gyroZ,accelX,accelY,accelZ"
        )

        self.get_logger().info(
            f"Wheel circumference: {WHEEL_CIRC:.4f} m"
        )

        self.get_logger().info(
            f"Track width: {TRACK_WIDTH:.4f} m"
        )

        self.get_logger().info(
            f"Counts/wheel revolution: "
            f"{COUNTS_PER_WHEEL_REV:.1f}"
        )

        self.get_logger().info(
            f"IMU frame: {self.imu_frame}"
        )

        self.get_logger().info(
            f"Wheel odom: "
            f"{self.odom_frame} -> {self.base_frame}"
        )

    # ============================================================
    # ROS TIME
    # ============================================================

    def now_msg(self):

        return self.get_clock().now().to_msg()

    # ============================================================
    # RAW SERIAL CALLBACK
    # ============================================================

    def raw_callback(
        self,
        msg: String
    ):

        line = msg.data.strip()

        if not line:
            return

        parts = [
            p.strip()
            for p in line.split(",")
        ]

        try:

            # ====================================================
            # NEW TELEMETRY
            # ====================================================

            if parts[0] == "D":

                self.parse_telemetry(parts)

            # ====================================================
            # Ignore other Arduino messages
            # ====================================================

            else:

                return

        except (
            ValueError,
            IndexError
        ) as exc:

            self.get_logger().warning(
                f"Failed to parse serial line: "
                f"{line} ({exc})"
            )

    # ============================================================
    # ARDUINO TIMESTAMP
    # ============================================================

    def check_arduino_timestamp(
        self,
        arduino_ms: int
    ):
        """
        Check Arduino millis() progression.

        Handles:

            normal:
                1000 -> 1050 -> 1100

            uint32 rollover:
                4294967290 -> 4294967295 -> 0 -> 5

            reset:
                200000 -> 100
        """

        arduino_ms &= UINT32_MAX

        if self.last_arduino_ms is None:

            self.last_arduino_ms = arduino_ms

            return

        previous = self.last_arduino_ms

        delta = (
            arduino_ms - previous
        ) & UINT32_MAX

        # ========================================================
        # NORMAL FORWARD PROGRESSION
        # ========================================================

        if delta < UINT32_HALF:

            if arduino_ms < previous:

                self.timestamp_wraps += 1

                self.get_logger().info(
                    "Arduino millis() uint32 rollover: "
                    f"{previous} -> {arduino_ms}"
                )

            self.timestamp_reset_active = False

        # ========================================================
        # BACKWARD / RESET
        # ========================================================

        else:

            self.timestamp_backwards += 1

            if not self.timestamp_reset_active:

                self.get_logger().warning(
                    "Arduino millis() moved backwards: "
                    f"previous={previous}, "
                    f"current={arduino_ms}"
                )

                self.timestamp_reset_active = True

        self.last_arduino_ms = arduino_ms

    # ============================================================
    # MAIN TELEMETRY PARSER
    # ============================================================

    def parse_telemetry(
        self,
        parts
    ):

        # ========================================================
        # EXPECTED:
        #
        # D
        # timestamp
        # LR
        # RR
        # yaw
        # pitch
        # roll
        # gyroX
        # gyroY
        # gyroZ
        # accelX
        # accelY
        # accelZ
        #
        # TOTAL = 13 fields
        # ========================================================

        if len(parts) < 13:

            self.get_logger().warning(
                "D telemetry has too few fields: "
                f"{len(parts)}"
            )

            return

        # ========================================================
        # PARSE FIELDS
        # ========================================================

        arduino_ms = int(
            float(parts[1])
        )

        lr_count = int(
            float(parts[2])
        )

        rr_count = int(
            float(parts[3])
        )

        yaw_deg = float(
            parts[4]
        )

        pitch_deg = float(
            parts[5]
        )

        roll_deg = float(
            parts[6]
        )

        gyro_x = float(
            parts[7]
        )

        gyro_y = float(
            parts[8]
        )

        gyro_z = float(
            parts[9]
        )

        accel_x = float(
            parts[10]
        )

        accel_y = float(
            parts[11]
        )

        accel_z = float(
            parts[12]
        )

        # ========================================================
        # TIMESTAMP CHECK
        # ========================================================

        self.check_arduino_timestamp(
            arduino_ms
        )

        # ========================================================
        # PUBLISH IMU
        # ========================================================

        self.publish_imu(
            yaw_deg,
            pitch_deg,
            roll_deg,
            gyro_x,
            gyro_y,
            gyro_z,
            accel_x,
            accel_y,
            accel_z
        )

        # ========================================================
        # UPDATE WHEEL ODOMETRY
        # ========================================================

        self.update_wheel_odometry(
            arduino_ms,
            lr_count,
            rr_count
        )

    # ============================================================
    # IMU PUBLISHER
    # ============================================================

    def publish_imu(
        self,
        yaw_deg,
        pitch_deg,
        roll_deg,
        gyro_x,
        gyro_y,
        gyro_z,
        accel_x,
        accel_y,
        accel_z
    ):

        # --------------------------------------------------------
        # Degrees -> radians
        # --------------------------------------------------------

        yaw = math.radians(
            yaw_deg
        )

        pitch = math.radians(
            pitch_deg
        )

        roll = math.radians(
            roll_deg
        )

        # --------------------------------------------------------
        # Quaternion
        # --------------------------------------------------------

        qx, qy, qz, qw = (
            euler_to_quaternion(
                roll,
                pitch,
                yaw
            )
        )

        # --------------------------------------------------------
        # ROS IMU message
        # --------------------------------------------------------

        imu = Imu()

        imu.header.stamp = (
            self.now_msg()
        )

        imu.header.frame_id = (
            self.imu_frame
        )

        # --------------------------------------------------------
        # Orientation
        #
        # Currently published for diagnostics.
        # Custom EKF can ignore it if required.
        # --------------------------------------------------------

        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw

        # -1 means orientation covariance is unknown
        imu.orientation_covariance[0] = -1.0

        # --------------------------------------------------------
        # Gyroscope
        # --------------------------------------------------------

        imu.angular_velocity.x = gyro_x
        imu.angular_velocity.y = gyro_y
        imu.angular_velocity.z = gyro_z

        gyro_variance = max(
            self.gyro_std ** 2,
            1e-12
        )

        imu.angular_velocity_covariance[0] = (
            gyro_variance
        )

        imu.angular_velocity_covariance[4] = (
            gyro_variance
        )

        imu.angular_velocity_covariance[8] = (
            gyro_variance
        )

        # --------------------------------------------------------
        # Accelerometer
        # --------------------------------------------------------

        imu.linear_acceleration.x = accel_x
        imu.linear_acceleration.y = accel_y
        imu.linear_acceleration.z = accel_z

        accel_variance = max(
            self.accel_std ** 2,
            1e-12
        )

        imu.linear_acceleration_covariance[0] = (
            accel_variance
        )

        imu.linear_acceleration_covariance[4] = (
            accel_variance
        )

        imu.linear_acceleration_covariance[8] = (
            accel_variance
        )

        # --------------------------------------------------------
        # Publish
        # --------------------------------------------------------

        self.imu_pub.publish(
            imu
        )

        self.imu_count += 1

    # ============================================================
    # WHEEL ENCODER -> ODOMETRY
    # ============================================================

    def update_wheel_odometry(
        self,
        arduino_ms: int,
        lr_count: int,
        rr_count: int
    ):

        # ========================================================
        # FIRST MESSAGE
        # ========================================================

        if self.previous_lr_count is None:

            self.previous_lr_count = lr_count
            self.previous_rr_count = rr_count
            self.previous_arduino_ms = arduino_ms

            return

        # ========================================================
        # ENCODER DELTAS
        # ========================================================

        delta_lr = (
            lr_count
            - self.previous_lr_count
        )

        delta_rr = (
            rr_count
            - self.previous_rr_count
        )

        # Save current counts
        self.previous_lr_count = lr_count
        self.previous_rr_count = rr_count

        # ========================================================
        # TIME DELTA
        # ========================================================

        previous_ms = (
            self.previous_arduino_ms
        )

        # Handle uint32 millis rollover
        dt_ms = (
            arduino_ms - previous_ms
        ) & UINT32_MAX

        self.previous_arduino_ms = (
            arduino_ms
        )

        if dt_ms <= 0:

            return

        dt = dt_ms / 1000.0

        # Reject impossible / bad time intervals
        if dt <= 0.0001 or dt > 1.0:

            return

        # ========================================================
        # COUNTS -> DISTANCE
        # ========================================================

        # One complete wheel revolution:
        #
        # COUNTS_PER_WHEEL_REV counts
        #
        # and the wheel travels:
        #
        # WHEEL_CIRC meters
        #

        meters_per_count = (
            WHEEL_CIRC
            /
            COUNTS_PER_WHEEL_REV
        )

        left_distance = (
            delta_lr
            * meters_per_count
        )

        right_distance = (
            delta_rr
            * meters_per_count
        )

        # ========================================================
        # DIFFERENTIAL DRIVE ODOMETRY
        # ========================================================

        # Average distance travelled
        center_distance = (
            left_distance
            + right_distance
        ) * 0.5

        # Change in heading
        delta_yaw = (
            right_distance
            - left_distance
        ) / TRACK_WIDTH

        # ========================================================
        # INTEGRATE POSE
        # ========================================================

        # Midpoint integration gives better accuracy
        # than using the old heading.

        midpoint_yaw = (
            self.odom_yaw
            + delta_yaw * 0.5
        )

        self.odom_x += (
            center_distance
            * math.cos(midpoint_yaw)
        )

        self.odom_y += (
            center_distance
            * math.sin(midpoint_yaw)
        )

        self.odom_yaw += delta_yaw

        self.odom_yaw = self.normalize_angle(
            self.odom_yaw
        )

        # ========================================================
        # VELOCITY
        # ========================================================

        linear_velocity = (
            center_distance / dt
        )

        angular_velocity = (
            delta_yaw / dt
        )

        # ========================================================
        # PUBLISH ODOMETRY
        # ========================================================

        self.publish_odometry(
            linear_velocity,
            angular_velocity
        )

    # ============================================================
    # ODOMETRY PUBLISHER
    # ============================================================

    def publish_odometry(
        self,
        linear_velocity,
        angular_velocity
    ):

        odom = Odometry()

        odom.header.stamp = (
            self.now_msg()
        )

        odom.header.frame_id = (
            self.odom_frame
        )

        odom.child_frame_id = (
            self.base_frame
        )

        # --------------------------------------------------------
        # Position
        # --------------------------------------------------------

        odom.pose.pose.position.x = (
            self.odom_x
        )

        odom.pose.pose.position.y = (
            self.odom_y
        )

        odom.pose.pose.position.z = 0.0

        # --------------------------------------------------------
        # Orientation
        # --------------------------------------------------------

        qx, qy, qz, qw = (
            euler_to_quaternion(
                0.0,
                0.0,
                self.odom_yaw
            )
        )

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # --------------------------------------------------------
        # Velocity
        # --------------------------------------------------------

        odom.twist.twist.linear.x = (
            linear_velocity
        )

        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0

        odom.twist.twist.angular.z = (
            angular_velocity
        )

        # ========================================================
        # COVARIANCE
        # ========================================================

        # Position uncertainty
        position_std = 0.05

        odom.pose.covariance[0] = (
            position_std ** 2
        )

        odom.pose.covariance[7] = (
            position_std ** 2
        )

        # Z position essentially unavailable
        odom.pose.covariance[14] = 99999.0

        # Roll / pitch unavailable
        odom.pose.covariance[21] = 99999.0
        odom.pose.covariance[28] = 99999.0

        # Yaw uncertainty
        yaw_std = math.radians(
            10.0
        )

        odom.pose.covariance[35] = (
            yaw_std ** 2
        )

        # Velocity covariance
        velocity_std = 0.05

        odom.twist.covariance[0] = (
            velocity_std ** 2
        )

        odom.twist.covariance[35] = (
            yaw_std ** 2
        )

        # --------------------------------------------------------
        # Publish
        # --------------------------------------------------------

        self.odom_pub.publish(
            odom
        )

        self.odom_count += 1

    # ============================================================
    # ANGLE NORMALIZATION
    # ============================================================

    @staticmethod
    def normalize_angle(
        angle
    ):

        while angle > math.pi:

            angle -= 2.0 * math.pi

        while angle <= -math.pi:

            angle += 2.0 * math.pi

        return angle


# ================================================================
# MAIN
# ================================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = SensorParser()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
