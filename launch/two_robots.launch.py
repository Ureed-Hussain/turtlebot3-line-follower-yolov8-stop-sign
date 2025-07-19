#!/usr/bin/env python3
"""
Launch two TurtleBot3 Burger‑Cam robots, each with its own namespace,
in the custom line‑following world, plus a namespaced line‑follower node
for each robot.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


# --------------------------------------------------------------------------- #
# Helper: build a robot + nodes group
# --------------------------------------------------------------------------- #
def robot_group(ns: str,            # namespace & Gazebo entity name
                x: float,
                y: float,
                yaw: float,
                urdf_path: str):

    return GroupAction([
        # Put everything below under /ns/…
        PushRosNamespace(ns),

        # Spawn robot (plugins will inherit namespace via -robot_namespace)
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", ns,
                "-database", "turtlebot3_waffle_pi",
                "-x", str(x), "-y", str(y), "-z", "0.15",
                "-Y", str(yaw),
                "-robot_namespace", ns
            ],
            output="screen"),

        # Joint and robot‑state publishers
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": open(urdf_path).read()}],
            output="screen"),

        # Line‑follower (relative topics resolve inside namespace)
        Node(
            package="turtlebot3_line_follower",
            executable="line_follower",
            name="line_follower",
            parameters=[{
                "camera_topic": "camera/image_raw",
                "cmd_topic":    "cmd_vel",
                # tweak gains if you like:
                # "forward_v": 0.10,
                # "k_omega":   0.01,
                # "white_thresh": 200
            }],
            output="screen"),
    ])


# --------------------------------------------------------------------------- #
# Main launch description
# --------------------------------------------------------------------------- #
def generate_launch_description():

    # Tell plugins / URDF which model to use
    os.environ["TURTLEBOT3_MODEL"] = "waffle_pi"

    # Path to Burger‑Cam URDF from turtlebot3_gazebo package
    tb3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    urdf_file = os.path.join(tb3_gazebo_dir, "urdf", "turtlebot3_waffle_pi.urdf")

    # Path to your custom world
    world_file = os.path.join(
        os.getenv("HOME"),
        "turtlebot3_line_follower",
        "src",
        "turtlebot3_line_follower",
        "worlds",
        "line_follow_world.world")

    return LaunchDescription([

        # 1 – Start Gazebo with the world
        ExecuteProcess(
            cmd=["gazebo", "--verbose", world_file, "-s", "libgazebo_ros_factory.so"],
            output="screen"),

        # 2 – Robot 1  (namespace /robot1)
        robot_group(
            ns="robot1",
            x=0.30,
            y=0.00,
            yaw=1.57,          #  +90 deg
            urdf_path=urdf_file),

        # 3 – Robot 2  (namespace /robot2)
        robot_group(
            ns="robot2",
            x=0.2861,
            y=-0.44051,
            yaw=-1.57,         #  −90 deg
            urdf_path=urdf_file),
        Node(
        package="turtlebot3_line_follower",
        executable="two_line_follower_node",
        name="two_line_follower_node",
        output="screen")
    ])
