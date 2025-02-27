## Introduction

This Python package provides a template for implementing swarm controllers within a Gazebo simulation environment integrated with PX4 and ROS2. It is based on [px4_swarm_controller](https://github.com/PX4/px4_swarm_controller) with several modifications—some functionalities have been changed and new features added—to better suit your swarm experimentation needs.

## Features

- **Swarm Control Template:** A starting point to design and test swarm algorithms.
- **Gazebo Integration:** Seamless simulation within a Gazebo environment.
- **PX4 Compatibility:** Works with the PX4 flight stack configured for ROS2.
- **ROS2 Communication:** Utilizes ROS2 nodes for command and telemetry exchange.

## Prerequisites

- **PX4 with ROS2 Integration:**  
  Follow the [PX4 ROS2 Communication Guide](https://docs.px4.io/main/en/ros/ros2_comm.html) for setup instructions.

- **ROS2 Humble Hawksbill:**  
  If ROS2 Humble is not installed, refer to the [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html).

## Installation

1. **Create Your ROS2 Workspace:**

   Open a terminal and run:

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **Clone the Repository:**

   Clone the `px4_swarm_controller_python` repository into your workspace:

   ```bash
   git clone https://github.com/jorgeafsilva/px4_swarm_controller_python
   ```

3. **Reorganize the Repository Structure:**

   Move the `custom_msgs` folder from inside the cloned repository to the `src` directory:

   ```bash
   mv px4_swarm_controller_python/custom_msgs .
   ```

6. **Build the Workspace:**

   Navigate to your workspace root and build all packages:

   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ```

7. **Source the Workspace:**

   After the build completes, source the setup file:

   ```bash
   source install/setup.bash
   ```

## Usage

- **Launching the Simulation:**

  To start the simulation, run:

  ```bash
  ros2 launch swarm_controller_python simulation.launch.py
  ```

## Customization

- **Controller Logic:**  
  Modify the Python scripts in the package to implement your custom swarm algorithms.

- **Integration:**  
  Extend the package by adding new ROS2 nodes or modifying existing messages to suit your application.

- **Simulation Parameters:**  
  Adjust Gazebo and PX4 parameters to match your simulation requirements.

## Contributing

Contributions, bug reports, and feature requests are welcome! Please open an issue or submit a pull request.

## License

This project is licensed under the BSD-3-Clause License. See the [LICENSE](LICENSE) file for details.
```

---
