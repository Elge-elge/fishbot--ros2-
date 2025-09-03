import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the URDF file
    urdf_package_path = get_package_share_directory('fishbot_description') 
    default_urdf_path = os.path.join(urdf_package_path, 'urdf','first_robot.urdf')
    default_rviz_config_path = os.path.join(urdf_package_path, 'config','display_robot_model.rviz')
    #声明urdf目录参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_urdf_path),
        description='加载的模型文件路径'
    )

    # 读取URDF文件内容，并传递给robot_state_publisher节点
    command_result = launch.substitutions.Command(['xacro ' , launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(command_result, value_type=str)

    # 启动robot_state_publisher节点
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}]
    )

    # 启动joint_state_publisher节点
    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    
    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d' , default_rviz_config_path],
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz_node,
    ])