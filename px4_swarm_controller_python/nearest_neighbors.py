import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from typing import List, Type
import numpy as np
from abc import ABC, abstractmethod
import math
from functools import partial
from rclpy.qos import DurabilityPolicy, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class NearestNeighbors(Node, ABC):
    """
    Python implementation of nearest neighbors detection for drone swarms.
    
    Subclasses must define:
    - NeighborsMsg: The ROS message type for neighbor information
    - Implement abstract methods for neighborhood processing
    """
    
    NeighborsMsg: Type = None  # To be defined by subclass
    
    def __init__(self):
        super().__init__("nearest_neighbors")
        
        # Parameter declarations
        self.declare_parameter("nb_drones", 1)
        self.declare_parameter("neighbor_distance", 5.0)
        self.declare_parameter("x_init", [0.0])
        self.declare_parameter("y_init", [0.0])
        
        # Parameter retrieval
        self.nb_drones = self.get_parameter("nb_drones").value
        self.neighbor_distance = self.get_parameter("neighbor_distance").value
        self.x_init = self.get_parameter("x_init").value
        self.y_init = self.get_parameter("y_init").value
        
        # Storage initialization
        self.position_received = [False] * self.nb_drones
        self.drones_positions = [VehicleLocalPosition() for _ in range(self.nb_drones)]
        self.neighbors_publishers = []
        self.position_subscribers = []

        # QOS for publishers and subscribers and so on
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create publishers and subscribers for each drone
        for i in range(self.nb_drones):
            # Publisher for neighbor information
            pub = self.create_publisher(
                self.NeighborsMsg,
                f"/px4_{i+1}/fmu/out/nearest_neighbors",
                10
            )
            self.neighbors_publishers.append(pub)
            
            # Subscriber for position updates
            sub = self.create_subscription(
                VehicleLocalPosition,
                f"/px4_{i+1}/fmu/out/vehicle_local_position",
                partial(self.pose_subscriber_callback, drone_idx=i),
                qos_profile
            )
            self.position_subscribers.append(sub)
        
        # Timer for periodic neighbor calculation (100ms)
        self.timer = self.create_timer(0.1, self.timer_callback)

    '''
    def create_position_callback(self, drone_idx):
        """Factory method to create position callback with drone index closure"""
        def callback(msg):
            self.pose_subscriber_callback(msg, drone_idx)
        return callback
    '''

    def pose_subscriber_callback(self, msg: VehicleLocalPosition, drone_idx: int):

        """Handle incoming position updates"""
        self.position_received[drone_idx] = True
        adjusted_pose = self.local_to_global(msg, drone_idx)
        self.drones_positions[drone_idx] = adjusted_pose

    def local_to_global(self, local_pose: VehicleLocalPosition, drone_idx: int) -> VehicleLocalPosition:
        """Convert local position to global coordinates"""
        adjusted = VehicleLocalPosition()
        adjusted.x = local_pose.x + self.x_init[drone_idx]
        adjusted.y = local_pose.y + self.y_init[drone_idx]
        adjusted.z = local_pose.z  # Z typically doesn't need offset

        '''
        if drone_idx == 0:
            self.get_logger().info(f"x={adjusted.x}, y={adjusted.y}, z={adjusted.z}")
        '''
            
        return adjusted

    def timer_callback(self):

        """Periodic neighbor calculation trigger"""
        if all(self.position_received):
            # Reset received flags
            self.position_received = [False] * self.nb_drones
            self.find_neighbors()

    def find_neighbors(self):
        """Main neighbor detection logic"""
        neighborhoods = []
        for i, position in enumerate(self.drones_positions): # build drone neighborhoods
            neighborhood = self.process_position(i, position)
            neighborhoods.append(neighborhood)
        
        self.process_global_variables()
        for i, neighborhood in enumerate(neighborhoods):
            if len(neighborhood.neighbors_position) > 0:
                self.enrich_neighborhood(neighborhood)
                self.neighbors_publishers[i].publish(neighborhood)

    def process_position(self, drone_idx: int, position: VehicleLocalPosition):
        """Process individual drone position to find neighbors"""
        neighborhood = self.NeighborsMsg()
        for neighbor_idx, neighbor_pos in enumerate(self.drones_positions):
            if self.is_neighbor(position, neighbor_pos, neighbor_idx == drone_idx):
                self.process_neighbor_position(
                    drone_idx,
                    neighbor_idx,
                    position,
                    neighbor_pos,
                    neighborhood
                )
        self.process_neighborhood(drone_idx, neighborhood)
        return neighborhood

    def is_neighbor(self, pos1: VehicleLocalPosition, pos2: VehicleLocalPosition, same_drone: bool) -> bool:
        """Determine if two positions are neighbors"""
        if same_drone:
            return False
            
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        dz = pos1.z - pos2.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        return 0.01 < distance <= self.neighbor_distance

    @abstractmethod
    def process_neighbor_position(self, drone_idx: int, neighbor_idx: int, 
                                 position: VehicleLocalPosition, neighbor_position: VehicleLocalPosition,
                                 neighborhood):
        """Abstract method to process individual neighbor relationships"""
        pass

    @abstractmethod
    def process_neighborhood(self, drone_idx: int, neighborhood):
        """Abstract method for final neighborhood processing"""
        pass

    @abstractmethod
    def process_global_variables(self):
        """Abstract method for processing additional global variables after building neighborhood messages"""
        pass

    @abstractmethod
    def enrich_neighborhood(self, neighborhood):
        """Abstract method for adding custom neighborhood data"""
        pass
