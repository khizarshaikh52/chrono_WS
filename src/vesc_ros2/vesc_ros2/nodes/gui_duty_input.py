import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import tkinter as tk

class DutyPublisher(Node):
    def __init__(self):
        super().__init__('duty_publisher')
        self.publisher_ = self.create_publisher(Float64, '/commands/motor/duty', 10)
        self.get_logger().info('DutyPublisher node started.')

    def publish_duty(self, value):
        try:
            duty = float(value)
            msg = Float64()
            msg.data = duty
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published duty: {duty}')
        except ValueError:
            self.get_logger().warn('Invalid input: please enter a numeric value.')

def main(args=None):
    rclpy.init(args=args)
    node = DutyPublisher()

    # --- GUI setup ---
    root = tk.Tk()
    root.title("VESC Duty Control")

    label = tk.Label(root, text="Enter duty cycle (-1.0 to 1.0):")
    label.pack(pady=10)

    entry = tk.Entry(root, width=10, font=('Arial', 14))
    entry.insert(0, "0.0")
    entry.pack(pady=5)

    def send_duty():
        node.publish_duty(entry.get())

    send_button = tk.Button(root, text="Send", command=send_duty, width=10, height=1)
    send_button.pack(pady=10)

    # Stop motor safely
    def stop_motor():
        entry.delete(0, tk.END)
        entry.insert(0, "0.0")
        node.publish_duty(0.0)

    stop_button = tk.Button(root, text="STOP", command=stop_motor, fg="white", bg="red", width=10, height=1)
    stop_button.pack(pady=5)

    # Exit cleanly
    def on_close():
        node.publish_duty(0.0)
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    # Run GUI + ROS spin
    def spin():
        rclpy.spin_once(node, timeout_sec=0.1)
        root.after(100, spin)

    spin()
    root.mainloop()

if __name__ == '__main__':
    main()
