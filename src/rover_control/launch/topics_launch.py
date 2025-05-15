from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from glob import glob
import subprocess

cam_1_serial = "728312070349"
cam_2_serial = "728312070350"


# The arducam camera could be in a different path, so this checks that
def find_arducam():
    for video_dev in glob('/dev/video'):
        try:
            output = subprocess.check_output(['udevadm', 'info', '--query=all', '--name', video_dev], encodings='utf-8')
            if 'ID_MODEL=Arducam_OV9281_USB_Camera' in output:
                return video_dev
        except subprocess.CalledProcessError:
            continue
    return None

def generate_launch_description():
    ld = LaunchDescription()

    # Will need to change '/dev/ttyUSB0' to whatever port the teensy is (I think the command is lsusb or something like that to look at the ports but i don't remember)
    ld.add_action(ExecuteProcess(
        cmd=['/home/lunabotics/Downloads/arduino-1.8.19-linuxaarch64/arduino-1.8.19/hardware/tools/teensy_reboot'],
        output='screen'
    ))

    # Start micro_ros
    ld.add_action(ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyACM0'
        ],
        output='screen'
    ))

    # Start the cameras
    ld.add_action(Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera1',
        name='rs_camera1',
        parameters=[{'serial_no': '033322071026'}],
        output='screen'
    ))
    ld.add_action(Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera2',
        name='rs_camera2',
        parameters=[{'serial_no': '048522075108'}],
        output='screen'
    ))

    # arducam_dev = find_arducam()
    # if arducam_dev:
    #     print("Found hopper bin cam")
    #     ld.add_action(Node(
    #         package='usb_cam',
    #         executable='usb_cam_node_exe',
    #         namespace='arducam',
    #         name='usb_cam',
    #         parameters=[
    #             {'video_device': arducam_dev},
    #             {'image_width': 640},
    #             {'image_height': 480},
    #             {'framerate': 30}
    #         ],
    #     ))
    # else:
    #     print("Did not find hopper bin cam")

    # Start the joystick node (not sure if i need this here but oh well)
    ld.add_action(Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    ))

    # Start the actual control node
    ld.add_action(Node(
        package='rover_control',
        executable='tibalt_control',
        name='tibalt_control',
        output='screen'
    ))

    # ld.add_action(Node(
    #     package='realsense2_camera',
    #     executable='realsense2_camera_node',
    #     name='cam_1',
    #     parameters=[{
    #         'serial_no': cam_1_serial,
    #         'camera_name': 'cam_1'
    #     }]
    # ))

    # ld.add_action(Node(
    #     package='realsense2_camera',
    #     executable='realsense2_camera_node',
    #     name='cam_2',
    #     parameters=[{
    #         'serial_no': cam_2_serial,
    #         'camera_name': 'cam_2'
    #     }]
    # ))

    return ld
