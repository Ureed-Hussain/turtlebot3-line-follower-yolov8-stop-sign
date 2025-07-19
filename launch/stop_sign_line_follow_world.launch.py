from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Set TurtleBot3 model
    os.environ['TURTLEBOT3_MODEL'] = 'burger_cam'

    # Paths
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    urdf_file = os.path.join(turtlebot3_gazebo_dir, 'urdf', 'turtlebot3_burger_cam.urdf')
    world_path = os.path.join(
        os.getenv('HOME'),
        'turtlebot3_line_follower',
        'src',
        'turtlebot3_line_follower',
        'worlds',
        'stop_sign.world'
    )

    return LaunchDescription([

        # 🌍 Launch Gazebo with custom world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 🤖 Spawn TurtleBot3 model
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3',
                '-database', 'turtlebot3_burger_cam',
                '-x', '0.277096', '-y', '0.012708', '-z', '0.149065',
                '-Y', '1.57'
            ],
            output='screen'
        ),

        # 🔧 Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # 🧠 Robot state publisher with URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file])
            }],
            output='screen'
        ),

        # 🛑 YOLO stop sign detection node
        Node(
            package='turtlebot3_line_follower',
            executable='yolo_detect',
            name='yolo_detect',
            output='screen',

        ),

    ])