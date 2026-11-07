from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='dsr01'),
        DeclareLaunchArgument('interval', default_value='2.0'),
        DeclareLaunchArgument('loop', default_value='false'),
        Node(
            package='e0509_gripper_demo',
            executable='gripper_demo',
            output='screen',
            parameters=[{
                'namespace': LaunchConfiguration('namespace'),
                'interval': LaunchConfiguration('interval'),
                'loop': LaunchConfiguration('loop'),
                # 순서를 바꾸려면 여기 수정: 'open' / 'close' / 0~700
                'steps': ['open', '200', '450', 'close', 'open'],
            }],
        ),
    ])
