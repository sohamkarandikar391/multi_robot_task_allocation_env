# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os
# import yaml

# def generate_launch_description():
#     pkg_dir = get_package_share_directory('mrta_simulation')
#     world_file = os.path.join(pkg_dir, 'worlds', 'simple_world.sdf')
#     config_path = os.path.join(pkg_dir, 'config', 'scenario_simple.yaml')

#     print(f"Loading world: {world_file}")
    
#     # 1. Start Gazebo
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             os.path.join(
#                 get_package_share_directory('ros_gz_sim'),
#                 'launch',
#                 'gz_sim.launch.py'
#             )
#         ]),
#         launch_arguments={'gz_args': f'-r {world_file}'}.items()
#     )
    
#     # 2. Bridge the "Create Entity" service (Required for spawning)
#     bridge_service = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         arguments=['/world/simple_world/create@ros_gz_interfaces/srv/SpawnEntity'],
#         output='screen'
#     )
    
#     # 3. The Spawner Node (Runs your python script)
#     spawn_scenario = TimerAction(
#         period=8.0,
#         actions=[
#             Node(
#                 package='mrta_simulation',
#                 executable='spawn_scenario',
#                 name='scenario_spawner',
#                 output='screen'
#             )
#         ]
#     )

#     # 4. DYNAMIC BRIDGE GENERATION
#     # We read the config file to know exactly which robots to bridge
#     bridge_arguments = []
    
#     print(f"Parsing config for bridges: {config_path}")
#     with open(config_path, 'r') as f:
#         config = yaml.safe_load(f)
        
#         if 'species' in config:
#             for species_name, data in config['species'].items():
#                 count = data.get('count', 0)
                
#                 for i in range(count):
#                     # Must match the naming logic in your Python Spawner
#                     robot_name = f"{species_name}_{i}"
                    
#                     # Because we fixed the SDF in the previous step, 
#                     # the topics are now Clean: /{robot_name}/cmd_vel
#                     # We bridge Gazebo (gz.msgs) <-> ROS (geometry_msgs)
                    
#                     # Twist (Velocity)
#                     bridge_arguments.append(
#                         f'/model/{robot_name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
#                     )
                    
#                     # Odometry
#                     bridge_arguments.append(
#                         f'/{robot_name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
#                     )
                    
#                     # TF (Optional, but good for visualization)
#                     bridge_arguments.append(
#                         f'/{robot_name}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
#                     )

#     # 5. Create ONE efficient bridge node for all robots
#     # (Running 1 node is better than running 10 separate nodes in a loop)
#     robot_bridge_node = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         name='all_robots_bridge',
#         arguments=[
#             # 1. Bridge the /model/ topic (Where the data actually lives in Gazebo)
#             f'/model/{species_name}_{i}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
            
#             # 2. Bridge the cmd_vel topic (Your existing logic likely handles this, or add it here)
#             # f'/model/{species_name}_{i}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            
#             for species_name, data in yaml.safe_load(open(config_path))['species'].items()
#             for i in range(data['count'])
#         ] + bridge_arguments, # Append any other arguments you generated in your loop
        
#         # 3. THE MAGIC TRICK: Rename '/model/X/odometry' to '/X/odometry'
#         remappings=[
#             # Odom Remapping (We fixed this earlier)
#             (f'/model/{species_name}_{i}/odometry', f'/{species_name}_{i}/odometry'),
            
#             # ADD THIS: Cmd_Vel Remapping
#             # Map ROS '/X/cmd_vel' -> GZ '/model/X/cmd_vel'
#             (f'/model/{species_name}_{i}/cmd_vel', f'/{species_name}_{i}/cmd_vel')
            
#             for species_name, data in yaml.safe_load(open(config_path))['species'].items()
#             for i in range(data['count'])
#         ],
#         output='screen'
#     )

#     robot_controllers = []

#     with open(config_path, 'r') as f:
#         config = yaml.safe_load(f)
#         for species_name, data in config['species'].items():
#             count = data.get('count', 0)
#             for i in range(count):
#                 robot_name = f"{species_name}_{i}"
#                 controller_node = Node(
#                     package='mrta_simulation',
#                     executable='robot_controller',
#                     name=f'controller_{robot_name}',
#                     arguments=[robot_name],
#                     output='screen'
#                 )
#                 robot_controllers.append(controller_node)

#     return LaunchDescription([
#         gazebo,
#         bridge_service,
#         spawn_scenario,
#         robot_bridge_node,
#     ])

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    pkg_dir = get_package_share_directory('mrta_simulation')
    world_file = os.path.join(pkg_dir, 'worlds', 'simple_world.sdf')
    config_path = os.path.join(pkg_dir, 'config', 'scenario_simple.yaml')
    
    print(f"Loading world: {world_file}")

    # 1. GENERATE BRIDGE ARGUMENTS & REMAPPINGS
    # We do this logic HERE, outside the Node definition, to avoid Syntax Errors.
    bridge_arguments = []
    bridge_remappings = []
    
    print(f"--- PARSING CONFIG FROM: {config_path} ---")
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
        
        if 'species' in config:
            for species_name, data in config['species'].items():
                count = data.get('count', 0)
                
                for i in range(count):
                    robot_name = f"{species_name}_{i}"
                    
                    # --- ARGUMENTS (Tell Bridge what topics to listen to) ---
                    # 1. CMD_VEL (ROS -> GZ)
                    # Bridge both formats to be safe
                    bridge_arguments.append(f'/model/{robot_name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist')
                    bridge_arguments.append(f'/{robot_name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist')
                    
                    # 2. ODOMETRY (GZ -> ROS)
                    # Bridge the /model/ topic (The Reliable Source)
                    bridge_arguments.append(f'/model/{robot_name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry')
                    
                    # 3. TF (Optional)
                    bridge_arguments.append(f'/model/{robot_name}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V')

                    # --- REMAPPINGS (Rename topics for the Controller) ---
                    # Rename /model/X/odometry -> /X/odometry
                    bridge_remappings.append(
                        (f'/model/{robot_name}/odometry', f'/{robot_name}/odometry')
                    )
                    # Rename /model/X/cmd_vel -> /X/cmd_vel (Ensure Gazebo gets the message)
                    bridge_remappings.append(
                        (f'/model/{robot_name}/cmd_vel', f'/{robot_name}/cmd_vel')
                    )

    # 2. GAZEBO SIMULATION
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )
    
    # 3. SERVICE BRIDGE (Needed for Spawning)
    bridge_service = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/simple_world/create@ros_gz_interfaces/srv/SpawnEntity'],
        output='screen'
    )
    
    # 4. ROBOT SPAWNER (Python Script)
    spawn_scenario = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='mrta_simulation',
                executable='spawn_scenario',
                name='scenario_spawner',
                output='screen'
            )
        ]
    )

    # 5. DYNAMIC ROBOT BRIDGE NODE
    # We pass the pre-calculated lists here
    robot_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='all_robots_bridge',
        arguments=bridge_arguments,
        remappings=bridge_remappings, # <--- Clean and Syntax Error Free
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge_service,
        spawn_scenario,
        robot_bridge_node,
    ])