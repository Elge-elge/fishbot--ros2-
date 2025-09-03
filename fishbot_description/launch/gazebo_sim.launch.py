#把urdf模型加载到gazebo中

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    #获取路径
    urdf_package_path = get_package_share_directory('fishbot_description') 
    default_xacro_path = os.path.join(urdf_package_path, 'urdf','fishbot/fishbot.urdf.xacro')
    default_gazebo_world_path = os.path.join(urdf_package_path, 'world','custom_room.world')
    #声明urdf目录参数
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_xacro_path),
        description='加载的模型文件路径'
    )

    # 读取URDF文件内容，并传递给robot_state_publisher节点
    command_result = launch.substitutions.Command(['xacro ' , launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(command_result, value_type=str)

    # 启动robot_state_publisher节点
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}],
    )

    # # 启动joint_state_publisher节点
    # action_joint_state_publisher = launch_ros.actions.Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    # )

    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments=[('world',default_gazebo_world_path),('verbose','true')] #传入world参数

    )
    
    # action_rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d' , default_rviz_config_path],
    # )

    # 启动spawn_entity.py节点，将机器人模型插入到gazebo中
    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'fishbot'],
    )

    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller fishbot_joint_state_broadcaster --set-state active'.split(' '),
        output='screen'
    )

    action_load_effort_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller fishbot_effort_controller --set-state active'.split(' '),
        output='screen'
    )

    action_load_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller fishbot_diff_drive_controller --set-state active'.split(' '),
        output='screen'
    )

    return launch.LaunchDescription([
        action_declare_arg_model_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        action_spawn_entity,

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[action_load_joint_state_controller],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_load_joint_state_controller,
                on_exit=[action_load_diff_drive_controller],
            )
        ),
    ])