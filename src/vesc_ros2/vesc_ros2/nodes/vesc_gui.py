import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import tkinter as tk

class DutyGUI(Node):
    def __init__(self):
        super().__init__('vesc_duty_gui')

        # Parameters
        self.declare_parameter('topic', '/commands/motor/duty')
        self.declare_parameter('max_abs_duty', 0.25)  # safety clamp

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.max_abs_duty = float(self.get_parameter('max_abs_duty').get_parameter_value().double_value)

        self.pub = self.create_publisher(Float64, self.topic, 10)

        # Tkinter UI
        self.root = tk.Tk()
        self.root.title('VESC Duty Slider (-1 .. +1)')

        tk.Label(self.root, text='Duty (-1 … +1)').pack(pady=6)

        # Slider from -1.00 to +1.00
        self.slider = tk.Scale(
            self.root,
            from_=-100, to=100, orient=tk.HORIZONTAL,
            length=420, showvalue=False, resolution=1,
            command=self.on_slider
        )
        self.slider.set(0)
        self.slider.pack(padx=12)

        # Readout label
        self.readout = tk.Label(self.root, text='Duty: 0.000 (clamped to ±{:.3f})'.format(self.max_abs_duty))
        self.readout.pack(pady=6)

        # Buttons
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(pady=8)

        tk.Button(btn_frame, text='STOP (0.0)', width=12, command=self.stop_now).pack(side=tk.LEFT, padx=6)
        tk.Button(btn_frame, text='Quit', width=12, command=self.on_quit).pack(side=tk.LEFT, padx=6)

        # Periodic re-publish so VESC keeps applying duty (20 Hz)
        self._current_cmd = 0.0
        self.root.after(50, self._tick)  # 50 ms

        # On window close, send zero
        self.root.protocol('WM_DELETE_WINDOW', self.on_quit)

    def clamp(self, x: float) -> float:
        m = self.max_abs_duty
        return max(-m, min(m, x))

    def on_slider(self, val_str):
        # Slider gives [-100..100]; map to [-1..1], then clamp to ±max_abs_duty
        s = float(val_str) / 100.0
        duty = self.clamp(s)
        self._current_cmd = duty
        self.readout.config(text=f'Duty: {s:.3f} (clamped → {duty:.3f})')

    def publish(self, duty: float):
        self.pub.publish(Float64(data=float(duty)))

    def _tick(self):
        # re-publish current command every 50 ms
        self.publish(self._current_cmd)
        self.root.after(50, self._tick)

    def stop_now(self):
        self.slider.set(0)
        self._current_cmd = 0.0
        self.publish(0.0)

    def on_quit(self):
        try:
            self.publish(0.0)
            self.get_logger().info('Sent duty 0.0, exiting GUI.')
        except Exception:
            pass
        self.root.destroy()

def main(args=None):
    rclpy.init(args=args)
    node = DutyGUI()
    try:
        node.root.mainloop()
    finally:
        # safety
        try:
            node.publish(0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
