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
    default_config_file_path = os.path.join(
        ament_index_python.packages.get_package_share_directory('grid_map_publisher'),
        'config',
        'params.yaml'
    )
    
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

    # Launch declarations
    declare_config_file_path_cmd = launch.actions.DeclareLaunchArgument(
        'grid_map_publisher_config_file',
        default_value = default_config_file_path,
        description = 'Full path to the grid_map_publisher config file'
    )

    grid_map_publisher_node = launch_ros.actions.LifecycleNode(package='grid_map_publisher', executable='grid_map_publisher_node',
        name='grid_map_publisher_node', namespace='', output='screen', emulate_tty=True, respawn=True,
        parameters = [{'config_file': launch.substitutions.LaunchConfiguration('grid_map_publisher_config_file')}])
    

    # Make the thermal camera node take the 'configure' transition
    grid_map_publisher_configure_trans_event = launch.actions.EmitEvent(
        event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(grid_map_publisher_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the  thermal camera node take the 'activate' transition
    grid_map_publisher_activate_trans_event = launch.actions.EmitEvent(
        event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(grid_map_publisher_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    active_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node = grid_map_publisher_node,
            start_state = 'configuring',
            goal_state = 'inactive',
            entities = [grid_map_publisher_activate_trans_event]
        )
    )

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
        declare_config_file_path_cmd,
        declare_visualization_config_file_cmd,
        declare_rviz_config_file_cmd,

        # Start Nodes
        grid_map_publisher_node,

        #activate system
        grid_map_publisher_configure_trans_event,
        active_state_handler,
        grid_map_visualization_node,
        rviz2_node
    ])