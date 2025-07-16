from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, TimerAction
def generate_launch_description():
    # 动态查找路径
    franka_package_share = get_package_share_directory('franka_twins')
    gripper_package_share = get_package_share_directory('gripper')
   

    # 子 launch 文件路径
    franka_launch_file = franka_package_share + '/launch/wave.launch.py'
    gripper_launch_file = gripper_package_share + '/launch/gripper_dual.launch.py'

    # 声明参数
    
   

    return LaunchDescription([
        
        
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gripper_launch_file)
        ),
        TimerAction(
            period=4.0,  # 延时秒数
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(franka_launch_file)
                )
            ]
        )
    ])