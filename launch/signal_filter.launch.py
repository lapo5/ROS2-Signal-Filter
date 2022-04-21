from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import sys
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.actions.execute_process import ExecuteProcess

def generate_launch_description():

    params = os.path.join(get_package_share_directory("signal_filter"), 'params', 'params.yaml')
    
    
    return LaunchDescription([
        
        Node(
            package='signal_filter',
            executable='filter',
            name='filter',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            parameters=[params],
        ),
])