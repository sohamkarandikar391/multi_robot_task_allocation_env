from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []
    
    for i in range(4):
        nodes.append(
            Node(
                package='mrta_simulation',
                executable='robot_controller',
                name=f'turtlebot_controller_{i}',
                arguments=[f'turtlebot_species_{i}'],
                output='screen'
            )
        )
    
    for i in range(4):
        nodes.append(
            Node(
                package='mrta_simulation',
                executable='robot_controller',
                name=f'crazyflie_controller_{i}',
                arguments=[f'crazyflie_species_{i}', '--aerial'],
                output='screen'
            )
        )
    
    return LaunchDescription(nodes)