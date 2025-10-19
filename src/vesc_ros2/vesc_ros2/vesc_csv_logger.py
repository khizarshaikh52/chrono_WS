import csv, os
from pathlib import Path
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from vesc_msgs.msg import VescStateStamped

class VesccsvLogger(Node):
    def __init__(self):
        super().__init__('vesc_csv_logger')
        # Parameters
        self.declare_parameter('output_dir', str(Path.home() / 'rosdata' / 'vesc_logs'))
        self.declare_parameter('topic_name', '/vesc/state')
        self.declare_parameter('filename_prefix', 'vesc_run')
        self.declare_parameter('append_datetime', True)
        self.declare_parameter('flush_every_n', 20)

        output_dir = Path(self.get_parameter('output_dir').get_parameter_value().string_value)
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        prefix     = self.get_parameter('filename_prefix').get_parameter_value().string_value
        append_dt  = self.get_parameter('append_datetime').get_parameter_value().bool_value
        self.flush_every_n = max(1, self.get_parameter('flush_every_n').get_parameter_value().integer_value)

        output_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now(timezone.utc).astimezone().strftime('%Y%m%d_%H%M%S') if append_dt else ''
        fname = f"{prefix}_{ts}.csv" if ts else f"{prefix}.csv"
        self.csv_path = output_dir / fname

        self.get_logger().info(f"Logging VESC state from '{topic_name}' to {self.csv_path}")
        self.f = open(self.csv_path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow([
            'ros_time_sec','ros_time_nanosec',
            'inp_voltage_V','avg_input_current_A','avg_motor_current_A',
            'duty_cycle','erpm','temperature_mos_C','temperature_motor_C'
        ])
        self._since_flush = 0

        self.sub = self.create_subscription(VescStateStamped, topic_name, self._cb, 10)

    def _cb(self, msg: VescStateStamped):
        s = msg.state
        row = [
            msg.header.stamp.sec, msg.header.stamp.nanosec,
            getattr(s, 'voltage_input', float('nan')),
            getattr(s, 'current_input', float('nan')),
            getattr(s, 'current_motor', float('nan')),
            getattr(s, 'duty_cycle', float('nan')),
            getattr(s, 'speed', float('nan')),
            getattr(s, 'temperature_pcb', float('nan')),
            getattr(s, 'temperature_motor', float('nan')),
        ]
        self.w.writerow(row)
        self._since_flush += 1
        if self._since_flush >= self.flush_every_n:
            self.f.flush(); os.fsync(self.f.fileno()); self._since_flush = 0

    def destroy_node(self):
        try:
            if hasattr(self, 'f') and not self.f.closed:
                self.f.flush(); os.fsync(self.f.fileno()); self.f.close()
        except Exception as e:
            self.get_logger().warn(f"CSV close error: {e}")
        super().destroy_node()

def main():
    rclpy.init()
    n = VesccsvLogger()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
