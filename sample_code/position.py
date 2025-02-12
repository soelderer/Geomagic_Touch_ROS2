import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tkinter as tk

class PoseSubscriber(Node):
    def __init__(self, refresh_rate_ms):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/phantom/pose',  # Replace with actual topic if different
            self.pose_callback,
            10  # QoS profile
        )

        # Create tkinter window
        self.window = tk.Tk()
        self.window.title("Device Position Display")

        # Set window size to 800x800
        self.canvas = tk.Canvas(self.window, width=800, height=800, bg='white')
        self.canvas.pack()

        # Define a circle radius and initial position
        self.circle_radius = 10  # Slightly larger circle for better visibility
        self.x_pos = 0
        self.z_pos = 0

        # Create an empty circle on the canvas
        self.circle = self.canvas.create_oval(self.x_pos - self.circle_radius,
                                              self.z_pos - self.circle_radius,
                                              self.x_pos + self.circle_radius,
                                              self.z_pos + self.circle_radius,
                                              fill="blue")

        # Define scaling factors for position values
        self.scale_factor = 1000  # Adjust scale factor for 800x800 canvas

        # Define the canvas center
        self.canvas_center_x = 400  # The center of canvas for X-coordinate (half of 800px)
        self.canvas_center_z = 400  # The center of canvas for Z-coordinate (half of 800px)

        # Store the refresh rate in seconds
        self.refresh_rate_ms = refresh_rate_ms / 1000.0  # Convert to seconds
        self.last_received_time = self.get_clock().now()  # Initialize using ROS2 time

    def pose_callback(self, msg: PoseStamped):
        current_time = self.get_clock().now()

        # Calculate time difference in nanoseconds
        time_diff_ns = (current_time - self.last_received_time).nanoseconds
        refresh_rate_ns = self.refresh_rate_ms * 1e9  # Convert milliseconds to nanoseconds

        # Process message only if interval has elapsed
        if time_diff_ns >= refresh_rate_ns:
            self.last_received_time = current_time

            # Extract x and z positions from the received message
            position = msg.pose.position

            # Scale x and z position to fit the new canvas size and center 0,0 in the middle
            self.x_pos = self.canvas_center_x + position.x * self.scale_factor
            self.z_pos = self.canvas_center_z - position.z * self.scale_factor  # Flip z-axis as z=0 should be in center

            # Update the position of the circle on the tkinter canvas
            self.canvas.coords(self.circle,
                               self.x_pos - self.circle_radius,
                               self.z_pos - self.circle_radius,
                               self.x_pos + self.circle_radius,
                               self.z_pos + self.circle_radius)

    def update_ros(self):
        """Process incoming messages while the GUI is active"""
        rclpy.spin_once(self, timeout_sec=0.1)  # Non-blocking spin with a small timeout
        self.window.after(int(self.refresh_rate_ms * 1000), self.update_ros)  # Schedule the update loop

    def run_gui(self):
        # Start the tkinter event loop in the main thread
        self.update_ros()  # Start spinning the node and updating the GUI
        self.window.mainloop()


def main(args=None):
    rclpy.init(args=args)
    # Set the desired refresh rate in milliseconds here (e.g., 20 ms)
    refresh_rate_ms = 20
    pose_subscriber = PoseSubscriber(refresh_rate_ms)
    pose_subscriber.run_gui()  # Run the GUI and ROS spin loop

if __name__ == '__main__':
    main()
