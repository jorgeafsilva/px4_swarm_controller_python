import rclpy
from rclpy.node import Node
from custom_msgs.msg import ReferenceSignal
import json
import time

class ReferenceSignalPublisher(Node):
    def __init__(self):
        super().__init__('reference_signal_publisher')

        # Declare parameters (expected from launch file)
        self.declare_parameter('wps', '[]')  # Default empty JSON array
        self.declare_parameter('duration', 5.0)

        # Get namespace (defaults to empty string if not set)
        self.namespace = self.get_namespace().strip('/')
        self.get_logger().info(f'Node namespace: {self.namespace}')

        # Read parameters
        self.wps = self.get_parameter('wps').value
        self.duration = self.get_parameter('duration').value

        # Parse waypoints (expecting a JSON string)
        try:
            self.waypoints = json.loads(self.wps)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid wps format! Expecting a JSON string.')
            self.waypoints = []

        if not self.waypoints:
            self.get_logger().warn('No waypoints received!')

        # Define publisher topic with namespace
        topic_name = f'{self.namespace}/reference_signal' if self.namespace else 'reference_signal'
        self.publisher = self.create_publisher(ReferenceSignal, topic_name, 10)

        # Publishing rate (10 Hz → every 0.1 seconds)
        self.timer_period = 0.1
        self.current_wp_index = 0
        self.start_time = time.time()

        self.timer = self.create_timer(self.timer_period, self.publish_reference)
        self.get_logger().info(f'Publishing to: {topic_name} at 10 Hz')

    def publish_reference(self):
        """Publishes waypoints as reference signals at 10 Hz and loops indefinitely."""
        # Check if we need to switch to the next waypoint
        elapsed_time = time.time() - self.start_time
        if elapsed_time >= self.duration:
            self.start_time = time.time()  # Reset timer
            self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)  # Loop to start

        # Get current waypoint
        wp = self.waypoints[self.current_wp_index]
        msg = ReferenceSignal()
        msg.reference = [float(wp['x']), float(wp['y']), float(wp['z']), float(wp['yaw'])]

        # Publish message
        self.publisher.publish(msg)
        # self.get_logger().info(f'Published reference: {msg.reference} (WP {self.current_wp_index + 1}/{len(self.waypoints)})')

def main(args=None):
    rclpy.init(args=args)
    node = ReferenceSignalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
