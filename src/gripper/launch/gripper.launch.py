from launch import LaunchDescription
from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='gripper',          # 实际的包名
#             executable='dh_gripper',  # 实际的可执行文件名
#             name='dh_gripper_node'       # 节点名字

#         )
#     ])
def generate_launch_description():
    return LaunchDescription([
        # Left gripper
        Node(
            package='gripper',
            executable='dh_gripper',
            namespace='right',
            name='dh_gripper_node',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 115200
            }]
        ),

        # Right gripper
        # Node(
        #     package='gripper',
        #     executable='dh_gripper',
        #     namespace='left',
        #     name='dh_gripper_node',
        #     parameters=[{
        #         'port': '/dev/ttyUSB1',
        #         'baudrate': 115200
        #     }]
        # ),
    ])