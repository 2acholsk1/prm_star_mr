from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    urdf = os.path.join('/home', 'ws/src/ackermann_vehicle/urdf/', 'ackermann_vehicle.sdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-file', 'ackermann_vehicle/urdf/ackermann_vehicle.sdf', '-entity', 'ackermann_vehicle'],
            output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace="/",
            output='screen',
            parameters=[{'use_sim_time': True,
                         'robot_description': robot_description}],
            remappings=remappings)
    ])
