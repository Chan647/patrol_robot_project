import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro 

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
    
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')

    xacro_file = os.path.join(turtlebot3_description_dir, 'urdf', f'turtlebot3_{turtlebot3_model}.urdf')
    
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(slam_toolbox_dir, 'config', 'mapper_params_online_async.yaml')
        }.items()
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_cartographer'), 
        'rviz', 'turtlebot3_cartographer.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node, 
        slam_launch,
        rviz_node
    ])