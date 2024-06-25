#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os, yaml, sys, json, re


port_list = os.popen("ls /dev | grep ttyUSB").read().split('\n')[:-1]
port_list = ["/dev/" + port for port in port_list]



data = {"id": "3",
        "initial_pose": [0.5, -0.5, 0.],
        "initial_orientation": [0., 1.57, 0],
        "port_list": port_list}

motor_name = "motor_" + data["id"]
data["id"] = data["id"] + "_"


params = [{key: data[key]} for key in data.keys()]

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = [str(data["initial_pose"][0]), str(data["initial_pose"][1]), str(data["initial_pose"][2]), str(data["initial_orientation"][0]), str(data["initial_orientation"][1]), str(data["initial_orientation"][2]), "1", "robot", motor_name]
        ),
        Node(package="lk_tech", executable="lk_tech_motor", name="Motor", namespace = "LK_TECH", parameters=[data], output='screen')
        ]
)