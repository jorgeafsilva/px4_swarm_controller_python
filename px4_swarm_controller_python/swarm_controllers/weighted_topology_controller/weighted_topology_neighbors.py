from px4_swarm_controller_python.nearest_neighbors import NearestNeighbors
from px4_msgs.msg import VehicleLocalPosition
from custom_msgs.msg import WeightedTopologyNeighbors
import numpy as np
import rclpy

class WeightedTopologyNeighbors(NearestNeighbors):
    """
    Custom neighbor detection with weighted topological relationships.
    Implement abstract methods to define weighting logic.
    """
    
    # Define your custom ROS message type for neighbor data
    NeighborsMsg = WeightedTopologyNeighbors  # Replace with your actual message type (e.g., WeightedNeighborsMsg)

    def __init__(self):
        super().__init__()
        # Add any additional initialization here
        # (e.g., parameters for weighting calculations)

        self.declare_parameter("leaders", [False] * self.nb_drones)
        self.leaders = self.get_parameter("leaders").value

        # self.prc_matrix = np.zeros(self.nb_drones, self.nb_drones)
        self.prc_vector = np.ones(self.nb_drones) * self.nb_drones # stores actual prc values
        self.next_prc_vector = np.zeros(self.nb_drones) # stores prc values for next iteration


    """
    CHANGE INPUT NEIGHBORHOOD (TYPE: WeightedTopologyNeighbors)
    PROCESSES ALL NEIGHBORS OF DRONE IDX i 
    """
    def process_neighbor_position(self, 
                                drone_idx: int, 
                                neighbor_idx: int, 
                                position: VehicleLocalPosition, # pose in global frame 
                                neighbor_position: VehicleLocalPosition, # pose in global frame
                                neighborhood):
        """
        Implement to:
        - Calculate weights based on positions
        - Store weights in the neighborhood message
        - Filter neighbors based on weights
        """
        if self.leaders[drone_idx]:
            if len(neighborhood.own_position) < 1:
                neighborhood.own_position = [position]

        neighborhood.neighbors_ids.append(neighbor_idx)
        neighbor_position_msg = VehicleLocalPosition()
        neighbor_position_msg.x = position.x - neighbor_position.x
        neighbor_position_msg.y = position.y - neighbor_position.y
        neighbor_position_msg.z = position.z - neighbor_position.z
        neighbor_position_msg.vx = position.vx - neighbor_position.vx
        neighbor_position_msg.vy = position.vy - neighbor_position.vy
        neighbor_position_msg.vz = position.vz - neighbor_position.vz
        neighborhood.neighbors_position.append(neighbor_position_msg)

    """
    CHANGE NEIGHBORHOOD AS A WHOLE AFTER IT HAS BEEN CHANGED BY process_neighbor_position()
    """
    def process_neighborhood(self, drone_idx: int, neighborhood):
        """
        Implement to:
        - Apply final weights/transformations
        - Sort neighbors by weight
        - Add topology metadata
        """

        # compute next prc of drone 
        if self.leaders[drone_idx]:
            self.next_prc_vector[drone_idx] = 1
        else:
            aux_prc_vector = [self.nb_drones] * self.nb_drones
            for j in range(len(aux_prc_vector)):
                aux_prc_vector[j] = self.prc_vector[j]
            self.next_prc_vector[drone_idx] = np.max([np.min(aux_prc_vector[drone_idx]) + 1, self.nb_drones])

        # compute weights
        weights = [] 
        sum_prcs = 0.

        for j in neighborhood.neighbors_ids:
            sum_prcs += 1 / self.prc_vector[j] 
        
        for j in neighborhood.neighbors_ids:
            weights.append((1 / self.prc_vector[j]) / sum_prcs)
        
        # add weights to neighborhood
        neighborhood.weights = weights
    
    # process global variables for next iteration
    def process_global_variables(self):
        self.prc_vector = np.copy(self.next_prc_vector)
    
    # not needed and overall redundant
    def enrich_neighborhood(self, neighborhood):
        """
        Implement to:
        - Add custom weights to the published message
        - Include additional topological relationships
        - Add quality metrics
        """

        pass

def main(args=None):
    rclpy.init(args=args)
    node = WeightedTopologyNeighbors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()