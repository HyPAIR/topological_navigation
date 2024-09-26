#!/usr/bin/env python3
""" Launch file for topological navigation.

Launches topological localisation, edge navigation, and map visualisation.

Author: Charlie Street
Owner: Charlie Street
"""

from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import launch


def generate_launch_description():

    # All launch args
    map_path = LaunchConfiguration("map")
    pose_topic = LaunchConfiguration("pose_topic")
    viz = LaunchConfiguration("viz")

    map_arg = DeclareLaunchArgument("map")
    pose_topic_arg = DeclareLaunchArgument("pose_topic", default_value="amcl_pose")
    viz_arg = DeclareLaunchArgument("viz", default_value="true")

    localisation_node = Node(
        package="topological_navigation",
        executable="topological_localisation",
        name="topological_localisation",
        parameters=[{"map": map_path}, {"pose_topic": pose_topic}],
        output="screen",
    )

    viz_node = Node(
        package="topological_navigation",
        executable="topological_map_visualiser",
        name="topological_map_visualiser",
        parameters=[{"map": map_path}],
        output="screen",
        condition=launch.conditions.IfCondition(
            launch.substitutions.LaunchConfiguration("viz")
        ),
    )

    # Add all the commands
    ld = LaunchDescription()
    ld.add_action(map_arg)
    ld.add_action(pose_topic_arg)
    ld.add_action(viz_arg)
    ld.add_action(localisation_node)
    ld.add_action(viz_node)

    return ld
