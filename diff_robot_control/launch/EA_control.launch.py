from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('diff_robot_description') + '/launch/display.launch.py'))
    dust_publisher_node = Node(
        package='diff_robot_control',
        executable='dust_particles_node',
        output='screen'
    )
    EA_velcity_control_node = Node(
        package='diff_robot_control',
        executable='velocity_evolutionary_algorithm',
        output='screen'
    )
    return LaunchDescription([
        simulation_launch,
        dust_publisher_node,
        EA_velcity_control_node
    ])
