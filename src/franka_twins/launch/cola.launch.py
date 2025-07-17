from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='franka_twins',          # 实际的包名
            executable='cola',  # 实际的可执行文件名
            name='franka_twins'       # 节点名字

        )
    ])