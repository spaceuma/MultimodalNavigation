import os
import launch
import launch_ros
import lifecycle_msgs
import launch
import launch.actions
import launch.events
import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle
import ament_index_python
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    visualization_config_file_path = os.path.join(
        ament_index_python.packages.get_package_share_directory('grid_map_publisher'),
        'config',
        'simple_demo.yaml'
    )

    rviz_config_file_path = os.path.join(
        ament_index_python.packages.get_package_share_directory('grid_map_publisher'),
        'config',
        'grid_map_demo.rviz'
    )
    
    # Declare launch configuration variables that can access the launch arguments values
    visualization_config_file = LaunchConfiguration('visualization_config')
    rviz_config_file          = LaunchConfiguration('rviz_config')
    
    # Declare launch arguments
    declare_visualization_config_file_cmd = DeclareLaunchArgument(
        'visualization_config',
        default_value = visualization_config_file_path,
        description   ='Full path to the Gridmap visualization config file to use')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value = rviz_config_file_path,
        description   = 'Full path to the RVIZ config file to use')

    grid_map_visualization_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[visualization_config_file]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )


    return LaunchDescription([
        # Set env var to print messages colored. The ANSI color codes will appear in a log.
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        declare_visualization_config_file_cmd,
        declare_rviz_config_file_cmd,

        # Start Nodes
        grid_map_visualization_node,
        rviz2_node
    ])