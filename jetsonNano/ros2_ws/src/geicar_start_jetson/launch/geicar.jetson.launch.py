from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    lidar_node = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        emulate_tty=True
    )

    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        emulate_tty=True
    )

    lane_detection_node = Node(
        package="lane_detection",
        executable="lane_detection_node",
        emulate_tty=True
    )
    
    lane_crossing_assist_node = Node(
        package="lane_detection",
        executable="lane_crossing_assist_node",
        emulate_tty=True
    )
    
    system_check_ack_node = Node(
        package="system_check_ack",
        executable="system_check_ack_node",
        emulate_tty=True
    )


    ld.add_action(lidar_node)
    ld.add_action(camera_node)
    ld.add_action(lane_detection_node)
    ld.add_action(lane_crossing_assist_node)
    ld.add_action(system_check_ack_node)

    return ld