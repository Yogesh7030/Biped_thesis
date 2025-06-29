from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution
import yaml
import os

def generate_launch_description():
	pkg_share = FindPackageShare('biped_description').find('biped_description')
	# controller_yaml_path = PathJoinSubstitution([
    # 	FindPackageShare("biped_description"),
    # 	"config",
    # 	"biped_controllers.yaml"
	# ])
	controller_yaml_file = os.path.join(pkg_share, 'config', 'biped_controllers.yaml')
	with open(controller_yaml_file, 'r') as f:
		full_yaml = yaml.safe_load(f)
	controller_params = full_yaml.get('controller_manager', {}).get('ros__parameters', {})
	urdf_path = pkg_share + '/urdf/biped.urdf.xacro'

	return LaunchDescription([
		Node(
			package='robot_state_publisher',
			executable='robot_state_publisher',
			name='robot_state_publisher',
			output='screen',
			parameters=[{
				'robot_description': Command(['xacro ', urdf_path])
			}]
		),
		
		ExecuteProcess(
			cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
			output='screen'
		),
		ExecuteProcess(
			cmd=[
				'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
				'-topic', 'robot_description',
				'-entity', 'biped2'
			],
			output='screen'
		),

		Node(
        	package='controller_manager',
        	executable='ros2_control_node',
			output='screen',
        	parameters=[
				{'robot_description': Command(['xacro ', urdf_path])},
				{'use_sim_time': True},
				controller_params
			]
    	),
		TimerAction(
			period=3.0,
			actions=[
				Node(
					package='controller_manager',
					executable='spawner',
					arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
					output='screen'
				)
			]
		),
		TimerAction(
			period=6.0,
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
