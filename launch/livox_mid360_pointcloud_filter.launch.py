# self_filter.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Get the path to the URDF file from the description package
    description_package_path = get_package_share_directory('description')
    urdf_file_path = os.path.join(description_package_path, 'urdf', 'scorpio8.urdf.xacro')

    # Process the xacro file to get robot description
    robot_description_content = Command(['xacro ', urdf_file_path])

    # Get the path to the YAML config file from robot_self_filter package
    robot_self_filter_package_path = get_package_share_directory('robot_self_filter')
    filter_config_file = os.path.join(robot_self_filter_package_path, 'params', 'livox_mid360_pointcloud_filter.yaml')

    # Create a log action to print the config
    log_config = LogInfo(msg=filter_config_file)

    self_filter_node = Node(
        package='robot_self_filter',
        executable='self_filter',
        name='livox_mid360_pointcloud_filter',
        output='screen',
        parameters=[
            filter_config_file,  # loads the YAML file
            {
                'robot_description': ParameterValue(robot_description_content, value_type=str),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        log_config,
        self_filter_node
    ])
