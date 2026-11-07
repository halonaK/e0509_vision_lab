"""
카메라 실습 launch (로봇 bringup 과 별도로 실행)

  ros2 launch e0509_camera_lab camera_lab.launch.py                    # 가상 RealSense
  ros2 launch e0509_camera_lab camera_lab.launch.py camera:=realsense  # 실제 RealSense

  띄우는 것
    - calib_tf_publisher : base_link -> camera_link (캘리브레이션 파일 또는 초기 추정값)
    - 카메라            : virtual_camera  또는  realsense2_camera (align_depth 켬)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera = LaunchConfiguration('camera')
    is_virtual = IfCondition(PythonExpression(["'", camera, "' == 'virtual'"]))
    is_rs = IfCondition(PythonExpression(["'", camera, "' == 'realsense'"]))
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='virtual', description='virtual | realsense'),
        DeclareLaunchArgument('calib_file', default_value='~/doosan_ws/eye_to_hand.yaml'),

        Node(package='e0509_camera_lab', executable='calib_tf_publisher', output='screen',
             parameters=[{'calib_file': LaunchConfiguration('calib_file')}]),

        Node(package='e0509_camera_lab', executable='virtual_camera', output='screen',
             condition=is_virtual),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
            launch_arguments={'align_depth.enable': 'true',
                              'publish_tf': 'true'}.items(),
            condition=is_rs),
    ])
