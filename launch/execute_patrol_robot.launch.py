from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    gui = ExecuteProcess(
        cmd=['python3', '/home/cho/gui.py'],
        output='screen'
    )

    patrol_node = Node(
        package='turtle_pkg',
        executable='patrol_robot_node',
        output='screen'
    )

    launchfile = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtle_pkg'),
                'launch',
                'slam_local.launch.py'
            ])
        )
    )

    return LaunchDescription([
        gui,
        TimerAction(period=1.0, actions=[patrol_node]),
        TimerAction(period=4.0, actions=[launchfile]),
    ])
