# File: pid_control_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'pid_control'
    package_share_directory = get_package_share_directory(package_name)
    config_file = os.path.join(package_share_directory, 'config', 'control_param.yaml')

    # Example node: 'input' (your setpoint generator)
    input_node = Node(
        package=package_name,
        executable='input',  # Matches the entry_point in setup.py
        name='input',
        output='screen',
        parameters=[config_file]
    )

    # Visualization / Debugging tools:
    # (Uncomment or remove these if you don't need them)
    rqt_graph_node = ExecuteProcess(
        cmd=["ros2", "run", "rqt_graph", "rqt_graph"],
        output="screen"
    )

    rqt_reconfigure_node = ExecuteProcess(
        cmd=["ros2", "run", "rqt_reconfigure", "rqt_reconfigure"],
        output="screen"
    )

    plotjuggler_node = ExecuteProcess(
        cmd=["ros2", "run", "plotjuggler", "plotjuggler"],
        output="screen"
    )

    return LaunchDescription([
        # Nodes in your system:
        input_node,
        
        # Optional debugging/visualization:
        rqt_graph_node,
        rqt_reconfigure_node,
        plotjuggler_node
    ])
