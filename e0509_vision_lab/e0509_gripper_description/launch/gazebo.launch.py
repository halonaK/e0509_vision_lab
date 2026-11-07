import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_path = get_package_share_directory('e0509_gripper_description')
    rh_pkg_path = get_package_share_directory('rh_p12_rn_a_description')
    dsr_pkg_path = get_package_share_directory('dsr_description2')
    
    xacro_file = os.path.join(pkg_path, 'urdf', 'e0509_with_gripper.urdf.xacro')

    ARGUMENTS = [
        DeclareLaunchArgument('name', default_value='e0509_gripper'),
    ]

    # Gazebo 리소스 경로
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ':',
            os.path.dirname(rh_pkg_path),
            ':',
            os.path.dirname(dsr_pkg_path),
        ]
    )

    # Robot description
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' use_gazebo:=true',
        ' namespace:=', LaunchConfiguration('name'),
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': '-r -v 3 empty.sdf'}.items(),
    )

    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='screen',
        parameters=[robot_description],
    )

    # Spawn robot
    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=LaunchConfiguration('name'),
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'e0509_gripper',
            '-allow_renaming', 'true',
        ],
    )

    # Spawners (without --param-file)
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            namespace=LaunchConfiguration('name'),
            arguments=['joint_state_broadcaster', '-c', 'controller_manager', '--param-file', os.path.join(get_package_share_directory('e0509_gripper_description'), 'config', 'gz_controllers.yaml')],
        )]
    )

    arm_controller_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            namespace=LaunchConfiguration('name'),
            arguments=['joint_trajectory_controller', '-c', 'controller_manager', '--param-file', os.path.join(get_package_share_directory('e0509_gripper_description'), 'config', 'gz_controllers.yaml')],
        )]
    )

    gripper_controller_spawner = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            namespace=LaunchConfiguration('name'),
            arguments=['gripper_controller', '-c', 'controller_manager', '--param-file', os.path.join(get_package_share_directory('e0509_gripper_description'), 'config', 'gz_controllers.yaml')],
        )]
    )

    # 컨트롤러가 켜지기 전 몇 초 동안 팔이 처지므로, 켜진 뒤 홈 자세로 세움
    go_home = TimerAction(
        period=7.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once',
                 ['/', LaunchConfiguration('name'), '/joint_trajectory_controller/joint_trajectory'],
                 'trajectory_msgs/msg/JointTrajectory',
                 "{joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], "
                 "points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}"],
            output='screen',
        )]
    )

    return LaunchDescription(ARGUMENTS + [
        gz_resource_path,
        gazebo,
        robot_state_pub,
        gz_spawn,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        go_home,
    ])
