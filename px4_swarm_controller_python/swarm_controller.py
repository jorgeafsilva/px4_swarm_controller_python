from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode
from enum import Enum
from typing import Type

class CONTROL(Enum):
    """Enum representing control modes for offboard control."""
    POSITION = 0
    VELOCITY = 1
    ACCELERATION = 2

class SwarmController(Node, ABC):
    """
    Base class for implementing swarm controllers for drone fleets in Python.
    
    Subclasses must specify the NeighborsMsg class attribute and implement
    the abstract callback methods.
    """
    
    # Subclasses must override this with the appropriate ROS message type
    NeighborsMsg: Type = None

    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        
        # Check if NeighborsMsg is properly defined
        if self.NeighborsMsg is None:
            raise NotImplementedError("Subclass must define NeighborsMsg class attribute")
        
        namespace = self.get_namespace()
        
        # Publisher for offboard control mode
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            f"{namespace}/fmu/in/offboard_control_mode",
            10
        )
        
        # Publisher for trajectory setpoints
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            f"{namespace}/fmu/in/trajectory_setpoint",
            10
        )
        
        # Subscriber for neighbor information
        self.neighbors_subscriber = self.create_subscription(
            self.NeighborsMsg,
            f"{namespace}/fmu/out/nearest_neighbors",
            self.neighbors_callback,
            10
        )
        
        # Timer for periodic control updates (100ms)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def publish_offboard_control_mode(self, control: CONTROL):
        """Publish the offboard control mode message."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        
        if control == CONTROL.POSITION:
            msg.position = True
        elif control == CONTROL.VELOCITY:
            msg.velocity = True
        elif control == CONTROL.ACCELERATION:
            msg.acceleration = True
            
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # Convert to μs
        self.offboard_control_mode_publisher.publish(msg)

    @abstractmethod
    def neighbors_callback(self, neighbors):
        """Abstract method to handle neighbor updates."""
        pass

    @abstractmethod
    def timer_callback(self):
        """Abstract method for periodic control updates."""
        pass
    