from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('turtlebot_nav')
    default_world = os.path.join(pkg_share, 'worlds', 'small.world')

    world_arg = DeclareLaunchArgument('world', default_value=default_world)

    # 1. Launch Gazebo Server (Physics Engine) - Use gzserver for stability
    gzserver = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world')],
        output='screen',
        emulate_tty=True
    )

    # 2. Launch Gazebo Client (GUI) - Use gzclient for the window
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        emulate_tty=True
    )

    robot_xacro = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description = Command(['xacro ', robot_xacro])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    # 3. Spawn Entity (Delayed to wait for Gazebo to load the world)
    spawn = TimerAction(
        period=3.0,  # Wait 3 seconds
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'turtlebot_nav', '-x', '1.0', '-y', '1.0', '-z', '0.0'],
                output='screen'
            )
        ]
    )

    # 4. Trainer Node (Delayed further to ensure sensor topics are active)
    trainer = TimerAction(
        period=5.0, # Wait 5 seconds
        actions=[
            Node(
                package='turtlebot_nav',
                executable='trainer_node.py',
                name='trainer_node',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        world_arg,
        gzserver,
        gzclient,
        rsp_node,
        spawn,
        trainer
    ])
