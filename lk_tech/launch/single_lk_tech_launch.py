#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os, yaml, sys, json, re


data = {"id": "1",
        "initial_pose": [0., 1.5, 0.]}

data["id"] = data["id"] + "_"
params = [{key: data[key]} for key in data.keys()]

def generate_launch_description():
    return LaunchDescription([Node(package="lk_tech", executable="lk_tech_motor", name="Motor", namespace = "LK_TECH", parameters=[data], output='screen')]
)