from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import ament_index_python
import os

def generate_launch_description():
    # Thermal camera node
    thermal_camera_dir         = ament_index_python.packages.get_package_share_directory('thermal_camera')
    thermal_camera_launch_dir  = os.path.join(thermal_camera_dir, 'launch')

    # Realsense camera node
    realsense_camera_dir        = ament_index_python.packages.get_package_share_directory('realsense_camera')
    realsense_camera_launch_dir = os.path.join(realsense_camera_dir, 'launch')

    return LaunchDescription([
        # Include the thermal camera launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(thermal_camera_launch_dir, 'thermal_camera.launch.py')),
        ),

        # Include the Realsense camera launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(realsense_camera_launch_dir, 'realsense_camera.launch.py')),
        ),
    ])