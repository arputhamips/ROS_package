import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
import xacro

def generate_launch_description():
    # ========================================================================
    # 1. SETUP & PATHS
    # ========================================================================
    description_pkg = 'robot_v1_description'
    bringup_pkg = 'robot_bringup'

    # Path to Xacro (Robot Description)
    xacro_file = os.path.join(get_package_share_directory(description_pkg), 'urdf', 'robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # Path to EKF Config
    ekf_config_path = os.path.join(get_package_share_directory(bringup_pkg), 'config', 'ekf.yaml')

    # ========================================================================
    # 2. NODES
    # ========================================================================

    # A. Robot State Publisher (The Body)
    # Publishes the static transforms (base_link -> camera, base_link -> imu)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # B. Micro-ROS Agent (The Muscles)
    # Connects to Teensy via the stable symlink we created
    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 
            'serial', '--dev', '/dev/teensy_main', '-b', '6000000'
        ],
        output='screen'
    )

    # C. Robot Localization (The Balance/Fusion)
    # Fuses Odom + IMU to create the smooth "odom" frame
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('/odometry/filtered', '/odom')]
    )

    # D. Camera Node (The Eyes)
    # Launches the Pi Camera V2 using the standard camera_ros package.
    # We explicitly link it to the 'camera_link_optical' frame defined in Xacro.
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'camera': '0',               # /dev/video0 or libcamera index 0
            'width': 640,                # Lower res is better for SLAM/Nav2 performance
            'height': 480,
            'frame_id': 'camera_link_optical', # CRITICAL: Matches sensors.xacro
            'format': 'UYVY',            # Standard for Pi Cams
        }]
    )

    # ========================================================================
    # 3. LAUNCH DESCRIPTION
    # ========================================================================
    return LaunchDescription([
        # Start the Agent first to establish hardware link
        micro_ros_agent,
        
        # Start the State Publisher to give structure
        robot_state_publisher_node,
        
        # Start the Sensors (Camera)
        camera_node,
        
        # Start the Fusion (EKF) - Added a small delay to let others settle? 
        # Usually not needed, but safe to launch immediately.
        ekf_node
    ])