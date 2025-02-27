import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode
from px4_swarm_controller_python.swarm_controller import SwarmController
from px4_swarm_controller_python.swarm_controller import CONTROL
from custom_msgs.msg import WeightedTopologyNeighbors
from custom_msgs.msg import ReferenceSignal
from px4_swarm_controller_python.second_order_filter import SecondOrderSmoother
import numpy as np
# from px4_msgs.msg import Neighbors  # Update with your actual message type

class WeightedTopologyController(SwarmController):

    NeighborsMsg = WeightedTopologyNeighbors

    def __init__(self):
        super().__init__(node_name='weighted_topology_controller')

        # Add your custom initialization here
        self.get_logger().info("Weighted Topology Controller initialized")

        # Get agent type (leader or follower)
        self.declare_parameter('agent_type', 0)
        self.agent_type = self.get_parameter('agent_type').value

        if self.agent_type == 1:
            self.reference_subscriber = self.create_subscription(
                ReferenceSignal,
                f"{self.get_namespace()}/reference_signal",
                self.reference_callback,
                10
            )
            self.x_filter = SecondOrderSmoother(alpha=0.5)
            self.y_filter = SecondOrderSmoother(alpha=0.5)
            self.z_filter = SecondOrderSmoother(alpha=0.5)
            self.vx_filter = SecondOrderSmoother(alpha=0.5)
            self.vy_filter = SecondOrderSmoother(alpha=0.5)
            self.vz_filter = SecondOrderSmoother(alpha=0.5)
            self.x_ref = None
            self.y_ref = None
            self.z_ref = None
            self.vx_ref = None
            self.vy_ref = None
            self.vz_ref = None

    # Required abstract method implementations
    def neighbors_callback(self, neighbors):
        """Handle incoming neighbor data"""
        self.get_logger().debug(f"Received neighbors: {neighbors}")
        # Implement your neighbor processing logic here
        # Example: store neighbor positions
        self.neighbors_position = neighbors.neighbors_position
        self.neighbor_weights = neighbors.weights
        if self.agent_type == 1:
            self.own_position = neighbors.own_position[0]

    def timer_callback(self):
        """Periodic control update"""
        # Implement your control logic here
        try:
            # Example control logic
            setpoint = self.calculate_weighted_setpoint()
            self.publish_setpoint(setpoint)
        except AttributeError as e:
            self.get_logger().error(f"Missing required data: {e}")

    # Add your custom methods
    def calculate_weighted_setpoint(self):
        """Example control calculation method"""
        # Implement your weighted topology logic here
        if self.agent_type == 0: # For followers
            setpoint = TrajectorySetpoint()  # Use your actual message type
            setpoint.acceleration = [0.0, 0.0, 0.0]
            setpoint.timestamp = self.get_clock().now().nanoseconds // 1000
        elif self.agent_type == 1: # For leaders
            setpoint = TrajectorySetpoint() 
            setpoint.timestamp = self.get_clock().now().nanoseconds // 1000
            if self.x_ref is None:
                setpoint.acceleration = [0.0, 0.0, 0.0]
                return setpoint
            neighbor_term_x = np.array([0., 0])
            neighbor_term_y = np.array([0., 0])
            neighbor_term_z = np.array([0., 0])
            for weight in self.neighbor_weights:
                neighbor_term_x = neighbor_term_x + weight * np.array([self.neighbors_position.x, self.neighbors_position.vx])
                neighbor_term_y = neighbor_term_y + weight * np.array([self.neighbors_position.y, self.neighbors_position.vy])
                neighbor_term_z = neighbor_term_z + weight * np.array([self.neighbors_position.z, self.neighbors_position.vz])
            y_x = neighbor_term_x + np.array([self.own_position.x - self.x_ref, self.own_position.vx - self.vx_ref])
            y_y = neighbor_term_y + np.array([self.own_position.y - self.y_ref, self.own_position.vy - self.vy_ref])
            y_z = neighbor_term_z + np.array([self.own_position.z - self.z_ref, self.own_position.vz - self.vz_ref])
            
            
        return setpoint

    # Store recevied reference
    def reference_callback(self, reference_msg):
        self.x_ref = self.x_filter.update((reference_msg[0]))
        self.y_ref = self.y_filter.update((reference_msg[1]))
        self.z_ref = self.z_filter.update((reference_msg[2]))
        self.vx_ref = self.vx_filter.update((reference_msg[3]))
        self.vy_ref = self.vy_filter.update((reference_msg[4]))
        self.vz_ref = self.vz_filter.update((reference_msg[5]))


    def publish_setpoint(self, setpoint):
        """Example publishing method"""
        self.trajectory_setpoint_publisher.publish(setpoint)
        self.publish_offboard_control_mode(CONTROL.ACCELERATION)

def main(args=None):
    rclpy.init(args=args)
    node = WeightedTopologyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()