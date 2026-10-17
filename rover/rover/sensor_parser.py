# sensor_parser.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
import numpy as np

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

class SensorParser(Node):
    def __init__(self):
        super().__init__('sensor_parser')
        
        self.sub_raw = self.create_subscription(String, 'raw_serial', self.raw_callback, 10)
        self.pub_imu = self.create_publisher(Imu, 'imu/data', 10)
        self.pub_odom = self.create_publisher(Odometry, 'wheel_odom', 10)

    def raw_callback(self, msg):
        line = msg.data.strip()
        
        # Rimuove il prefisso 'data:' se presente
        if line.startswith("data:"):
            line = line.replace("data:", "", 1).strip()
            
        # Pulisce gli spazi attorno a ciascun valore separato da virgola
        parts = [p.strip() for p in line.split(',')]

        if not parts or not parts[0]:
            return

        header = parts[0]
        now = self.get_clock().now().to_msg()

        try:
            # Formato IMU: IMU,<time>,<yaw>,<pitch>,<roll>,<gx>,<gy>,<gz>,<ax>,<ay>,<az>
            if header == "IMU" and len(parts) >= 11:
                imu_msg = Imu()
                imu_msg.header.stamp = now
                imu_msg.header.frame_id = "imu_link"

                yaw = math.radians(float(parts[2]))
                pitch = math.radians(float(parts[3]))
                roll = math.radians(float(parts[4]))

                q = euler_to_quaternion(roll, pitch, yaw)
                imu_msg.orientation.x = q[0]
                imu_msg.orientation.y = q[1]
                imu_msg.orientation.z = q[2]
                imu_msg.orientation.w = q[3]

                imu_msg.angular_velocity.x = float(parts[5])
                imu_msg.angular_velocity.y = float(parts[6])
                imu_msg.angular_velocity.z = float(parts[7])

                imu_msg.linear_acceleration.x = float(parts[8])
                imu_msg.linear_acceleration.y = float(parts[9])
                imu_msg.linear_acceleration.z = float(parts[10])

                self.pub_imu.publish(imu_msg)

            # Formato ODOM: ODOM,<poseX>,<poseY>,<poseYaw>
            elif header == "ODOM" and len(parts) >= 4:
                odom_msg = Odometry()
                odom_msg.header.stamp = now
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_link"

                odom_msg.pose.pose.position.x = float(parts[1])
                odom_msg.pose.pose.position.y = float(parts[2])

                yaw = math.radians(float(parts[3]))
                q = euler_to_quaternion(0, 0, yaw)
                odom_msg.pose.pose.orientation.x = q[0]
                odom_msg.pose.pose.orientation.y = q[1]
                odom_msg.pose.pose.orientation.z = q[2]
                odom_msg.pose.pose.orientation.w = q[3]

                self.pub_odom.publish(odom_msg)

            elif header == "ENC" and len(parts) >= 5:
                pass

        except ValueError as e:
            self.get_logger().debug(f"Failed to parse line: {line} - Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorParser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
