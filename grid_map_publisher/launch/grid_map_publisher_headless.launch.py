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


    return LaunchDescription([
        # Set env var to print messages colored. The ANSI color codes will appear in a log.
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        declare_config_file_path_cmd,
        
        # Start Nodes
        grid_map_publisher_node,

        #activate system
        grid_map_publisher_configure_trans_event,
        active_state_handler,
    ])