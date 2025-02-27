#!/usr/bin/env python3
import json
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

'''
def generate_launch_description():

    ld = LaunchDescription()
    package_dir = get_package_share_directory('px4_swarm_controller_python')

    with open(os.path.join(package_dir, 'config', 'swarm_config.json'), 'r') as swarm_file:
        swarm_config_data = json.load(swarm_file)
    
    with open(os.path.join(package_dir, 'config', 'control_config.json'), 'r') as control_file:
        control_config = json.load(control_file)
'''

def parse_swarm_config(config_file):
    # Extract unique models and their counts
    model_counts = {}
    swarm_config = config_file["swarm"]
    for key, item in swarm_config.items():
        model = item["model"]
        model_counts[model] = model_counts.get(model, 0) + 1
    script = ""
    for key, item in model_counts.items():
        script += key + ":" + str(item) + ","
    script = script[:-1]
    # Extract all initial poses
    initial_poses_string = "\""
    initial_poses_dict = dict()
    for idx, item in enumerate(swarm_config.values()):
        initial_pose = item["initial_pose"]
        initial_poses_dict["px4_" + str(idx + 1)] = initial_pose
        initial_poses_string += str(initial_pose["x"]) + "," + str(initial_pose["y"]) + "|"
    initial_poses_string = initial_poses_string[:-1] + "\""
    # Extract all is_leader values
    is_leaders = [item["is_leader"] for item in swarm_config.values()]
    return len(swarm_config), script, initial_poses_string, initial_poses_dict, is_leaders, config_file["trajectory"]


def parse_trajectory_yaml(yaml_file):
    """ Reads a trajectory YAML file and extracts waypoints + duration. """
    try:
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading YAML file: {e}")
        return {}, 5.0  # Default empty waypoints & duration

    # Extract waypoints for each namespace (e.g., `/px4_1`)
    keys = []
    for key in data:
        if "/" in key:
            keys.append(key)
    wps_per_namespace = {key: data[key]["wps"] for key in keys}
    durations = {key: data[key].get("duration", 5.0) for key in keys}  # Default duration: 5s

    return wps_per_namespace, durations

def generate_launch_description():

    ld = LaunchDescription()
    package_dir = get_package_share_directory('px4_swarm_controller_python')

    with open(os.path.join(package_dir, 'config', 'swarm_config.json'), 'r') as swarm_file:
        swarm_config_data = json.load(swarm_file)
    nb_drones, script, initial_poses, initial_poses_dict, is_leaders, trajectory_file = parse_swarm_config(swarm_config_data)
    swarm_file.close()

    with open(os.path.join(package_dir, 'config', 'control_config.json'), 'r') as control_file:
        control_config = json.load(control_file)

    # Read the trajectory YAML file
    trajectory_yaml_path = os.path.join(package_dir, 'config', "Trajectories/" + trajectory_file)
    wps_per_namespace, durations = parse_trajectory_yaml(trajectory_yaml_path)

    print(wps_per_namespace)
    print(durations)

    # Neighborhood and formation parameters
    neighborhood = control_config["neighborhood"]
    neighbors_exe, neighbors_distance, neighbors_params = neighborhood["neighbors_exe"], \
        neighborhood["neighbor_distance"], neighborhood["params"]

    # Control parameters
    controller_info = control_config["controller"]
    controller_exe, controller_params, is_leader_follower_control = controller_info["controller_exe"], controller_info[
        "params"], controller_info["leader_follower"]
    control_file.close()

    if is_leader_follower_control:
        neighbors_params = {"leaders": is_leaders, **neighbors_params}
    else:
        is_leaders = [False for is_leader in is_leaders]

    xs_init = []
    ys_init = []
    # Add trajectory generator if the drone is a leader and a controller otherwise
    for (namespace, initial_pose), aleader in zip(initial_poses_dict.items(), is_leaders):
        # For all the following nodes, we will switch the x-axis and the y-axis to follow the North East Down
        # convention. In fact, to spawn the drones, we need to follow Gazebo's frame convention East North Up.
        # Therefore, we need to switch the x-axis and the y-axis if we want to respect PX4's conventions.

        xs_init.append(initial_pose["y"])
        ys_init.append(initial_pose["x"])
        

        ## LAUNCH CONTROLLERS 
        if aleader:
            ld.add_action(Node(
                package='px4_swarm_controller_python',
                executable=controller_exe,
                name=controller_exe,
                namespace=namespace,
                # Gains: [Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y, Kp_z, Ki_z, Kd_z]
                parameters=[{"gains": controller_params["gains"], "agent_type": 1}] # Add stuff to the dictionary
            ))
        else:
            ld.add_action(Node(
                package='px4_swarm_controller_python',
                executable=controller_exe,
                name=controller_exe,
                namespace=namespace,
                # Gains: [Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y, Kp_z, Ki_z, Kd_z]
                parameters=[controller_params] # No need to add dictionary agent type as follower is the default agent type
            ))

    # Launching nearest neighbors
    ld.add_action(
        Node(
            package='px4_swarm_controller_python',
            executable=neighbors_exe,
            name='nearest_neighbors',
            namespace='simulation',
            # X,Y,Z formations represents the position wanted in relation to the leader
            parameters=[
                {"nb_drones": nb_drones, "neighbor_distance": neighbors_distance,
                 "x_init": xs_init, "y_init": ys_init, **neighbors_params}]
        ))

    # Launching simulation
    ld.add_action(
        Node(
            package='px4_swarm_controller_python',
            executable='simulation_node',
            name='simulation_node',
            parameters=[{'script': script, 'initial_pose': initial_poses}]
        )
    )

    ld.add_action(
        Node(
            package='px4_swarm_controller',
            executable='arming',
            name='arming',
            namespace='simulation',
            parameters=[{"nb_drones": nb_drones}]
        ))

    # Launch Reference Signal Publishers for Each Namespace
    for namespace in wps_per_namespace.keys():
        wps_json = json.dumps(wps_per_namespace[namespace])  # Convert to JSON string
        duration = durations.get(namespace, 5.0)  # Default to 5s if not found

        ld.add_action(Node(
            package='px4_swarm_controller_python',
            executable='reference_signal_publisher',
            name='reference_signal_publisher',
            namespace=namespace,
            parameters=[
                {'wps': wps_json},
                {'duration': duration}
            ]
        ))

    return ld