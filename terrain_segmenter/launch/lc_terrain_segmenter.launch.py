import os
import launch
import launch_ros
import lifecycle_msgs
import launch.actions
import launch.events
import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    yaml_config = os.path.join(
        get_package_share_directory('terrain_segmenter'),
        'config',
        'params.yaml'
    )

    terrain_segmenter_node = launch_ros.actions.LifecycleNode(package='terrain_segmenter', 
        executable='lc_terrain_segmenter', name='lc_terrain_segmenter', namespace='', output='screen', emulate_tty=True, respawn=True,
        arguments=['--ros-args', '--log-level', "info"], parameters = [yaml_config])
    
    # Make the thermal camera node take the 'configure' transition
    terrain_segmenter_configure_event = launch.actions.EmitEvent(
        event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(terrain_segmenter_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the  thermal camera node take the 'activate' transition
    terrain_segmenter_trans_event = launch.actions.EmitEvent(
        event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(terrain_segmenter_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )


    active_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node = terrain_segmenter_node,
            start_state = 'configuring',
            goal_state = 'inactive',
            entities = [terrain_segmenter_trans_event]
        )
    )

    
    return LaunchDescription([
        # Set env var to print messages colored. The ANSI color codes will appear in a log.
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        # Start Nodes
        terrain_segmenter_node,

        #activate system
        terrain_segmenter_configure_event,
        active_state_handler,
    ])