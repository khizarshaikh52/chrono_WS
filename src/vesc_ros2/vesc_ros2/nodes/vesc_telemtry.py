#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, Int32
from diagnostic_msgs.msg import KeyValue
import time

# pyvesc VESC interface
from pyvesc import VESC

class VESCTelemetry(Node):
    def __init__(self):
        super().__init__('vesc_telemetry')
        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('poll_hz', 10.0)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.poll_dt = 1.0 / float(self.get_parameter('poll_hz').get_parameter_value().double_value)

        # Publishers
        self.pub_voltage = self.create_publisher(Float32, 'vesc/voltage', 10)
        self.pub_rpm     = self.create_publisher(Int32,   'vesc/rpm', 10)
        self.pub_current = self.create_publisher(Float32, 'vesc/current', 10)
        self.pub_duty    = self.create_publisher(Float32, 'vesc/duty', 10)
        self.pub_temp    = self.create_publisher(Float32, 'vesc/motor_temp', 10)
        self.pub_batt    = self.create_publisher(BatteryState, 'vesc/battery_state', 10)
        self.pub_diag    = self.create_publisher(KeyValue, 'vesc/diag', 10)

        self.vesc = None
        self._connect()

        self.timer = self.create_timer(self.poll_dt, self._tick)

    def _connect(self):
        try:
            self.get_logger().info(f'Connecting to VESC on {self.port}...')
            self.vesc = VESC(serial_port=self.port, baudrate=115200, timeout=0.2)
            self.get_logger().info('VESC connected.')
            kv = KeyValue()
            kv.key = "vesc_connected"
            kv.value = "true"
            self.pub_diag.publish(kv)
        except Exception as e:
            self.get_logger().warn(f'VESC connect failed: {e}')
            self.vesc = None

    def _tick(self):
        if self.vesc is None:
            # Try to reconnect occasionally
            self._connect()
            time.sleep(0.5)
            return

        try:
            m = self.vesc.get_measurements()
            # Voltage (battery)
            self.pub_voltage.publish(Float32(data=float(m.v_in)))
            # RPM
            self.pub_rpm.publish(Int32(data=int(m.rpm)))
            # Current (avg motor current)
            self.pub_current.publish(Float32(data=float(m.avg_motor_current)))
            # Duty cycle (0..1)
            self.pub_duty.publish(Float32(data=float(m.duty_cycle)))
            # Motor temp if available
            temp = float(getattr(m, "temp_motor", 0.0))
            self.pub_temp.publish(Float32(data=temp))

            # BatteryState (minimal fill)
            bs = BatteryState()
            bs.voltage = float(m.v_in)
            bs.current = float(m.avg_input_current) if hasattr(m, "avg_input_current") else 0.0
            bs.percentage = float('nan')  # unknown unless you track SOC
            bs.present = True
            self.pub_batt.publish(bs)

        except Exception as e:
            self.get_logger().warn(f'VESC read failed: {e}')
            try:
                if self.vesc:
                    self.vesc.serial.close()
            except Exception:
                pass
            self.vesc = None

def main():
    rclpy.init()
    node = VESCTelemetry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
