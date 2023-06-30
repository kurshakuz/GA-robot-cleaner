from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_share = FindPackageShare(
        package='diff_robot_description').find('diff_robot_description')
    default_model_path = os.path.join(
        pkg_share, 'description/diff_robot_description_circular.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    gazebo_models_path = os.path.join(pkg_share, 'world/models')
    world_path = os.path.join(pkg_share, 'world', 'small_house.world')
    # world_path = os.path.join(pkg_share, 'world', 'my_world.sdf')

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true',
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    sim_arg = DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                    description='Flag to enable use_sim_time')
    gazebo_proc = ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s',
                                      'libgazebo_ros_factory.so', str(world_path)],
                                 cwd=str(gazebo_models_path),
                                 additional_env={'GAZEBO_MODEL_PATH': str(gazebo_models_path)},
                                 output='screen')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'diff_robot', '-topic',
                   'robot_description', '-x', '4', '-y', '0', '-z', '0.2', '-Y', '0'],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        sim_arg,
        gazebo_proc,
        joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node,
    ])