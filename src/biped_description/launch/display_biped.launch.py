from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, TimerAction
import os

def generate_launch_description():
    pkg_share = FindPackageShare('biped_description').find('biped_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'biped.urdf.xacro')

    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path]),
                'use_sim_time': True
            }]
        ),

        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn robot into Gazebo
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-topic', 'robot_description',
                '-entity', 'biped'
            ],
            output='screen'
        ),

        # Spawn joint_state_broadcaster
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster','--controller-manager', '/controller_manager'],
                    output='screen'
                )
            ]
        ),
        
		# Spawn joint_position_controller
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_position_controller', '--controller-manager', '/controller_manager'],
                    output='screen'
                )
            ]
        )
    ])
