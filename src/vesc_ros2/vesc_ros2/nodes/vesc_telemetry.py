import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial
import struct

class VescTelemetryNode(Node):
    def __init__(self):
        super().__init__('vesc_telemetry')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.serial_port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(self.serial_port, self.baud, timeout=1)
            self.get_logger().info(f'Connected to VESC telemetry on {self.serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            return

        self.pub_voltage = self.create_publisher(Float64, '/sensors/voltage', 10)
        self.pub_rpm = self.create_publisher(Float64, '/sensors/rpm', 10)
        self.timer = self.create_timer(0.5, self.poll_data)

    def poll_data(self):
        try:
            # Simplified example — real VESC packets require CRC etc.
            # For testing connection only
            voltage = 14.8
            rpm = 1200.0
            self.pub_voltage.publish(Float64(data=voltage))
            self.pub_rpm.publish(Float64(data=rpm))
            self.get_logger().info(f'Voltage: {voltage}V | RPM: {rpm}')
        except Exception as e:
            self.get_logger().warn(f'Error reading data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VescTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
