from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Will need to change '/dev/ttyUSB0' to whatever port the teensy is (I think the command is lsusb or something like that to look at the ports but i don't remember)
        ExecuteProcess(
            cmd=['/home/lunabotics/Downloads/arduino-1.8.19-linuxaarch64/arduino-1.8.19/hardware/tools/teensy_reboot'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyACM0'
            ],
            output='screen'
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        Node(
            package='rover_control',
            executable='tibalt_control',
            name='tibalt_control',
            output='screen'
        )
    ])
