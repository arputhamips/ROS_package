# Groundbot ROS 2 Stack

This package serves as the main entry point for the **Agribots Ground Robot**. It acts as a meta-package containing both the high-level robot logic and the necessary hardware drivers.

This stack is designed to run in a distributed architecture:
* **Rover:** Raspberry Pi (Drivers, Hardware Interface, Micro-ROS Agent)
* **Base Station:** Laptop/Workstation (RTK Corrections, Visualization, Mission Control)

## 📂 Directory Structure

```text
ROS_package/
├── drivers/             # External hardware drivers (Git Submodules)
│   ├── bno085_driver/
│   └── rtk_gps_driver/
├── my_robot_bringup/    # Launch files and runtime configurations
└── my_robot_logic/      # Custom nodes (Path planning, decision making)

```



- In any deployment (linux or docker), this repo is symliunked to  ~/workspace/src/ROS_Package[the repo]

