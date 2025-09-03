import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the  file
    autopatrol_robot_path = get_package_share_directory('autopatrol_robot')
    defalut_patrol_config_path = os.path.join(autopatrol_robot_path, 'config','patrol_config.yaml')
    action_patrol_node = launch_ros.actions.Node(
        package='autopatrol_robot',
        executable='patrol_node',
        output='screen',
        parameters=[defalut_patrol_config_path]
    )
    
    action_speaker_node = launch_ros.actions.Node(
        package='autopatrol_robot',
        executable='speaker',
        output='screen',
    )
    

    return launch.LaunchDescription([
        action_patrol_node,
        action_speaker_node,
    ])