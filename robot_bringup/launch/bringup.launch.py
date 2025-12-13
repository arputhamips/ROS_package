import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # ========================================================================
    # 1. SETUP PATHS
    # ========================================================================
    description_pkg = 'robot_v1_description'
    bringup_pkg = 'robot_bringup'

    # Path to the EXISTING rsp.launch.py in your description package
    rsp_launch_path = os.path.join(
        get_package_share_directory(description_pkg), 
        'launch', 
        'rsp.launch.py'
    )

    # Path to your EKF config
    ekf_config_path = os.path.join(
        get_package_share_directory(bringup_pkg), 
        'config', 
        'ekf.yaml'
    )

    # ========================================================================
    # 2. DEFINE NODES & INCLUDES
    # ========================================================================

    # A. INCLUDE ROBOT STATE PUBLISHER (from robot_v1_description)
    # This automatically handles processing the URDF/Xacro file
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_path),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # B. MICRO-ROS AGENT
    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 
            'serial', '--dev', '/dev/teensy_main', '-b', '6000000'
        ],
        output='screen'
    )

    # C. ROBOT LOCALIZATION (EKF)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('/odometry/filtered', '/odom')]
    )

    # D. CAMERA NODE
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'camera': 0,
            'width': 640,
            'height': 480,
            'frame_id': 'camera_link_optical', 
            'format': 'UYVY',
        }]
    )

    # ========================================================================
    # 3. LAUNCH EVERYTHING
    # ========================================================================
    return LaunchDescription([
        rsp_launch,
        micro_ros_agent,
        ekf_node,
        camera_node
    ])