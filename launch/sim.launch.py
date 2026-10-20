"""Gazebo Fortress + Husky with wheel encoders, a top-centre lidar, an IMU and an EKF."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD_NAME = 'husky_world'   # must match <world name> in the .sdf
ROBOT_NAME = 'husky'


def generate_launch_description():
    pkg = get_package_share_directory('husky_sensor_sim')
    world_file = os.path.join(pkg, 'worlds', 'husky_world.sdf')
    xacro_file = os.path.join(pkg, 'urdf', 'husky.urdf.xacro')
    rviz_file = os.path.join(pkg, 'rviz', 'husky.rviz')
    ekf_file = os.path.join(pkg, 'config', 'ekf.yaml')

    use_rviz = LaunchConfiguration('rviz')
    use_ekf = LaunchConfiguration('ekf')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', WORLD_NAME,
            '-topic', 'robot_description',
            '-name', ROBOT_NAME,
            '-x', '0.0', '-y', '0.0', '-z', '0.02', '-Y', '0.0',
        ],
        output='screen',
    )

    # ROS <-> Gazebo topics.  '[' = Gazebo->ROS, ']' = ROS->Gazebo
    joint_state_topic = f'/world/{WORLD_NAME}/model/{ROBOT_NAME}/joint_state'
    ground_truth_topic = f'/model/{ROBOT_NAME}/odometry'
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            f'{joint_state_topic}@sensor_msgs/msg/JointState[ignition.msgs.Model',
            f'{ground_truth_topic}@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        remappings=[
            (joint_state_topic, '/joint_states'),
            (ground_truth_topic, '/ground_truth/odom'),
        ],
        output='screen',
    )

    # Sensor fusion: encoders + IMU -> /odometry/filtered
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_file, {'use_sim_time': True}],
        condition=IfCondition(use_ekf),
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('ekf', default_value='true', description='Start the EKF (encoders + IMU)'),
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        ekf,
        rviz,
    ])
