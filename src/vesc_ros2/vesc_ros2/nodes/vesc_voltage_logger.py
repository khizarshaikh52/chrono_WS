import rclpy
from rclpy.node import Node
from pyvesc import VESC
import csv, os, time


class VescTrueVoltageLogger(Node):
    def __init__(self):
        super().__init__('vesc_true_voltage_logger')

        # ----- parameters -----
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('outfile', os.path.expanduser('~/vesc_true_voltage_log.csv'))

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = int(self.get_parameter('baud_rate').get_parameter_value().integer_value)
        self.rate_hz = float(self.get_parameter('rate_hz').get_parameter_value().double_value)
        self.outfile = self.get_parameter('outfile').get_parameter_value().string_value

        # ----- connect to VESC -----
        try:
            self.vesc = VESC(serial_port=self.port, baudrate=self.baud, timeout=0.05)
            self.get_logger().info(f'Connected to VESC on {self.port} @ {self.baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open {self.port}: {e}')
            raise

        # ----- prepare CSV -----
        os.makedirs(os.path.dirname(self.outfile), exist_ok=True)
        self._fh = open(self.outfile, 'w', newline='')
        self._csv = csv.writer(self._fh)
        self._csv.writerow(['time_iso', 'time_unix', 'v_in'])
        self.get_logger().info(f'Logging real VESC voltage to {self.outfile}')

        # ----- timer -----
        period = 1.0 / max(1e-3, self.rate_hz)
        self.create_timer(period, self.poll)

    def poll(self):
        try:
            vals = self.vesc.get_values()      # query the VESC
            v_in = float(vals.get('v_in', float('nan')))
        except Exception as e:
            self.get_logger().warn(f'get_values() failed: {e}')
            return

        now = self.get_clock().now()
        unix_ts = now.nanoseconds / 1e9
        iso = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(unix_ts)) + \
              f'.{int(now.nanoseconds % 1e9):09d}Z'

        self._csv.writerow([iso, f'{unix_ts:.6f}', f'{v_in:.3f}'])
        self._fh.flush()
        self.get_logger().info(f'V_in = {v_in:.2f} V')

    def destroy_node(self):
        try:
            self.vesc.ser.close()
        except Exception:
            pass
        try:
            self._fh.flush()
            self._fh.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VescTrueVoltageLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
