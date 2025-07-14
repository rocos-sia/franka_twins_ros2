from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Left gripper
        Node(
            package='gripper',
            executable='dh_gripper',
            namespace='left',
            name='dh_gripper_node',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 115200
            }]
        ),

        # Right gripper
        Node(
            package='gripper',
            executable='dh_gripper',
            namespace='right',
            name='dh_gripper_node',
            parameters=[{
                'port': '/dev/ttyUSB1',
                'baudrate': 115200
            }]
        ),
    ])
