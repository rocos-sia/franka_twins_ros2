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
                'port': '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AB0NVD39-if00-port0',
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
                'port': '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG011J3L-if00-port0', 
                'baudrate': 115200
            }]
        ),
    ])
