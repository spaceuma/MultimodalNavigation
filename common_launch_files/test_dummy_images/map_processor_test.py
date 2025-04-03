from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import ament_index_python
import os

def generate_launch_description():
    # Image aligner node
    image_merger_dir         = ament_index_python.packages.get_package_share_directory('dummy_image_merger')
    image_merger_launch_dir  = os.path.join(image_merger_dir, 'launch')

    # Image aligner node
    image_aligner_dir        = ament_index_python.packages.get_package_share_directory('image_aligner')
    image_aligner_launch_dir = os.path.join(image_aligner_dir, 'launch')

    # Terrain segmenter
    terrain_segmenter_dir        = ament_index_python.packages.get_package_share_directory('terrain_segmenter')
    terrain_segmenter_launch_dir = os.path.join(terrain_segmenter_dir, 'launch')

    # Map processor
    map_processor_dir        = ament_index_python.packages.get_package_share_directory('map_processor')
    map_processor_launch_dir = os.path.join(map_processor_dir, 'launch')

    return LaunchDescription([
        # Include the image merger launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(image_merger_launch_dir, 'dummy_image_merger.launch.py')),
        ),

        # Include the image aligner launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(image_aligner_launch_dir, 'lc_image_aligner.launch.py')),
        ),

        # Include the terrain segmenter launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(terrain_segmenter_launch_dir, 'lc_terrain_segmenter.launch.py')),
        ),

        # Include the map processor launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(map_processor_launch_dir, 'lc_map_processor.launch.py')),
        ),
    ])