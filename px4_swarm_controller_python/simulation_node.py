#!/usr/bin/env python3

# Import the subprocess and time modules
import rclpy
from rclpy.node import Node
import subprocess
import time


class SimulationScript(Node):
    def __init__(self):
        super().__init__('simulation_node')
        self.declare_parameter('script', '')
        self.declare_parameter('initial_pose', '')
        (s, p) = self.get_parameters(
            ['script', 'initial_pose'])

        init_poses = []
        for init_pose in p.value.split("|"):
            init_poses.append(init_pose.strip("\""))

        commands = ["pkill -9 -f \"gz sim\"", "MicroXRCEAgent udp4 -p 8888"]
        vhc_nb = 0
        for string in s.value.split(","):
            model_name, model_nb = string.split(":")
            for _ in range(int(model_nb)):
                cmd = f"cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART={4001} PX4_GZ_MODEL_POSE=\"{init_poses[vhc_nb]}\" PX4_SIM_MODEL={model_name} ./build/px4_sitl_default/bin/px4 -i {vhc_nb+1}"         
                commands.append(cmd)
                self.get_logger().info(cmd)
                vhc_nb += 1

        # Loop through each command in the list
        for command, cmd_nb in zip(commands, range(len(commands))):
            # Each command is run in a new tab of the gnome-terminal
            subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])

            # Pause between each command
            if cmd_nb == 2:
                time.sleep(15)
            else:
                time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    node = SimulationScript()
    try:
        rclpy.spin(node)
    finally:
        subprocess.run("pkill gnome-terminal", shell=True)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
