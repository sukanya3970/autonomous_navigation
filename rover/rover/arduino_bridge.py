# arduino_bridge.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import threading

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # The Arduino code initializes Serial at 115200 baud.
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {port} at {baudrate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect: {e}")
            raise SystemExit
            
        # Subscribe to cmd_vel to send velocities to Arduino
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        
        # Publish raw serial lines for the parser node
        self.raw_pub = self.create_publisher(String, 'raw_serial', 50)
        
        # Start a dedicated thread for blocking serial reads
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

    def cmd_callback(self, msg):
        # The Arduino expects velocity commands as "v,w"[cite: 1].
        v = msg.linear.x
        w = msg.angular.z
        cmd_str = f"{v:.4f},{w:.4f}\n"
        
        if self.serial_port.is_open:
            self.serial_port.write(cmd_str.encode('utf-8'))

    def read_serial_loop(self):
        while rclpy.ok() and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        raw_msg = String()
                        raw_msg.data = line
                        self.raw_pub.publish(raw_msg)
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
