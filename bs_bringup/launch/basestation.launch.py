import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    # 1. Start RViz2
    # We don't load a config yet because you haven't saved one. 
    # Once you save a config to 'config/view.rviz', you can uncomment the arguments line.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('bs_bringup'), 'config', 'view.rviz')]
    )

    # 2. Start RQT Image View
    # Great for checking the compressed feed to save bandwidth
    rqt_image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        output='screen'
    )

    # 3. Teleop Twist Keyboard (in a NEW TERMINAL)
    # This tries to pop up a new 'gnome-terminal' window running the keyboard controller.
    # If you don't have gnome-terminal, this might fail (just run it manually).
    teleop_terminal = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
        output='screen'
    )

    # 4. [NEW] Mission Recorder
    # Automatically listens for "RUNNING" status to start recording
    recorder_node = Node(
        package='bs_controller',
        executable='mission_recorder',
        name='mission_recorder',
        output='screen'
    )

    return LaunchDescription([
        rviz_node,
        teleop_terminal,
        recorder_node
    ])