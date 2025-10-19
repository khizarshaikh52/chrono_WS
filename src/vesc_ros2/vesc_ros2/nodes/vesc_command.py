import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import serial, struct, threading, time, csv, os

# ----- VESC UART constants -----
COMM_SET_DUTY = 5
COMM_GET_VALUES = 4

def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def make_packet(cmd: int, payload: bytes) -> bytes:
    body = bytes([cmd]) + payload
    L = len(body)
    header = bytes([2, L])                     # 2 = short frame
    crc = crc16(body)
    tail = bytes([(crc >> 8) & 0xFF, crc & 0xFF, 3])  # 3 = end
    return header + body + tail

def duty_packet(duty: float) -> bytes:
    i = int(max(-0.95, min(0.95, duty)) * 100000.0)   # int32, duty * 1e5
    return make_packet(COMM_SET_DUTY, struct.pack('>i', i))

def get_values_packet() -> bytes:
    return make_packet(COMM_GET_VALUES, b'')


class VescCommandNode(Node):
    def __init__(self):
        super().__init__('vesc_command')

        # ---- params ----
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('duty_cycle_min', -0.25)   # allow reverse by default
        self.declare_parameter('duty_cycle_max',  0.25)
        self.declare_parameter('csv_path', os.path.expanduser('~/vesc_voltage_log.csv'))
        self.declare_parameter('telemetry_hz', 10.0)      # how often to read v_in
        self.declare_parameter('deadman_s', 0.4)          # stop if no cmd for this long

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = int(self.get_parameter('baud_rate').get_parameter_value().integer_value)
        self.dmin = float(self.get_parameter('duty_cycle_min').get_parameter_value().double_value)
        self.dmax = float(self.get_parameter('duty_cycle_max').get_parameter_value().double_value)
        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.telemetry_period = 1.0 / float(self.get_parameter('telemetry_hz').get_parameter_value().double_value)
        self.deadman_s = float(self.get_parameter('deadman_s').get_parameter_value().double_value)

        # ---- serial ----
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.02)
            self.get_logger().info(f'Connected to VESC on {self.port} @ {self.baud}')
        except Exception as e:
            self.get_logger().error(f'Could not open {self.port}: {e}')
            self.ser = None

        # ---- CSV ----
        try:
            os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        except Exception:
            pass
        self.csv_fh = open(self.csv_path, 'w', newline='')
        self.csv = csv.writer(self.csv_fh)
        self.csv.writerow(['time_iso', 'time_unix', 'duty_cmd', 'v_in'])
        self.get_logger().info(f'Logging to {self.csv_path}')

        # ---- state ----
        self.last_duty = 0.0
        self.last_cmd_time = time.time()
        self.latest_vin = float('nan')

        # ---- ROS sub ----
        self.create_subscription(Float64, '/commands/motor/duty', self.on_duty, 10)
        self.get_logger().info('Listening on /commands/motor/duty')

        # ---- threads ----
        self.stop_evt = threading.Event()
        self.tx_thread = threading.Thread(target=self.tx_loop, daemon=True)
        self.tx_thread.start()
        self.rx_thread = threading.Thread(target=self.telemetry_loop, daemon=True)
        self.rx_thread.start()

        # periodic logger (write a row even if duty unchanged)
        self.create_timer(1.0 / 20.0, self.log_row)  # 20 Hz log line

    # ---------- callbacks ----------
    def on_duty(self, msg: Float64):
        req = float(msg.data)
        clamped = max(self.dmin, min(self.dmax, req))
        if clamped != self.last_duty:
            self.get_logger().info(f'req {req:.3f} -> duty {clamped:.3f}')
        self.last_duty = clamped
        self.last_cmd_time = time.time()

    # ---------- loops ----------
    def tx_loop(self):
        period = 1.0 / 20.0  # keep streaming duty at 20 Hz
        pkt_zero = duty_packet(0.0)
        while not self.stop_evt.is_set():
            if self.ser:
                try:
                    duty_to_send = self.last_duty
                    if time.time() - self.last_cmd_time > self.deadman_s:
                        duty_to_send = 0.0
                    self.ser.write(duty_packet(duty_to_send))
                except Exception as e:
                    self.get_logger().warn(f'UART write failed: {e}')
            time.sleep(period)
        # on exit, send zero once more
        try:
            if self.ser:
                self.ser.write(pkt_zero)
        except Exception:
            pass

    def telemetry_loop(self):
        """Poll v_in using COMM_GET_VALUES and parse only v_in from the reply."""
        # We’ll do very lightweight parsing: find a valid framed packet and
        # extract v_in using the known order/scale from VESC firmware.
        # v_in is sent as a 16-bit value in 0.1 V units (deci-volts) in the
        # short-values frame (many firmwares); for safety we search for 2/3 framed packets.

        def read_packet():
            # read bytes until we see a full framed message (2 .. 3)
            # timeout is set on the serial; keep simple and robust
            # returns payload (without cmd) or None
            try:
                # send request
                if self.ser:
                    self.ser.write(get_values_packet())
                else:
                    return None

                # now read until we see a start byte 2
                start = self.ser.read(1)
                if start != b'\x02':
                    return None
                ln = self.ser.read(1)
                if len(ln) != 1:
                    return None
                L = ln[0]
                body = self.ser.read(L)
                if len(body) != L:
                    return None
                crc = self.ser.read(2)
                end = self.ser.read(1)
                if end != b'\x03':
                    return None
                # verify crc
                if crc16(body) != ((crc[0] << 8) | crc[1]):
                    return None
                return body  # first byte is command id
            except Exception:
                return None

        while not self.stop_evt.is_set():
            payload = read_packet()
            if payload and len(payload) > 1 and payload[0] == COMM_GET_VALUES:
                data = payload[1:]  # strip command id
                # Parse only v_in from the data block.
                # In the "values" message, v_in is encoded as int16 in deci-volts near the start
                # after several fields; for many VESC firmwares it sits at offset where
                # bytes represent 0.1 V. We attempt a heuristic scan for a plausible voltage.
                vin = self._extract_vin_heuristic(data)
                if vin is not None:
                    self.latest_vin = vin
            time.sleep(self.telemetry_period)

    def _extract_vin_heuristic(self, data: bytes):
        # Try sliding-window int16 big-endian looking for a plausible 4S LiPo voltage (10–18 V)
        for i in range(0, max(0, len(data) - 1)):
            val = (data[i] << 8) | data[i + 1]
            # interpret as signed?
            if val > 32767:
                val -= 65536
            # check plausible deci-volts 100..180
            if 100 <= val <= 180:
                return val / 10.0
        return None

    # ---------- logging ----------
    def log_row(self):
        now = self.get_clock().now()
        ts = now.nanoseconds / 1e9
        iso = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(ts)) + f'.{int(now.nanoseconds % 1e9):09d}Z'
        vin = self.latest_vin
        self.csv.writerow([iso, f'{ts:.6f}', f'{self.last_duty:.6f}', f'{vin if vin==vin else ""}'])
        # flush occasionally
        try:
            self.csv_fh.flush()
        except Exception:
            pass

    # ---------- teardown ----------
    def destroy_node(self):
        self.stop_evt.set()
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        try:
            self.csv_fh.flush()
            self.csv_fh.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VescCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
