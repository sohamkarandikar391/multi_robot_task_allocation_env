from setuptools import setup
from glob import glob
import os

package_name = 'mrta_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), 
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models', 'depot_marker'),
            glob('models/depot_marker/*')),
        (os.path.join('share', package_name, 'models', 'task_marker'),
            glob('models/task_marker/*')),
        (os.path.join('share', package_name, 'models', 'simple_turtlebot'),
            glob('models/simple_turtlebot/*')),
        (os.path.join('share', package_name, 'models', 'simple_quadrotor'),
            glob('models/simple_quadrotor/*')),
        (os.path.join('share', package_name, 'models', 'short_obstacle'),
            glob('models/short_obstacle/*')),
        (os.path.join('share', package_name, 'models', 'tall_obstacle'),  
            glob('models/tall_obstacle/*')),
        (os.path.join('share', package_name, 'models/simple_turtlebot_red'), 
            glob('models/simple_turtlebot_red/*')),
        (os.path.join('share', package_name, 'models/simple_turtlebot_blue'), 
            glob('models/simple_turtlebot_blue/*')),
        (os.path.join('share', package_name, 'models/simple_turtlebot_green'), 
            glob('models/simple_turtlebot_green/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sohamk3901',
    maintainer_email='your_email@example.com',
    description='MRTA Simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'spawn_scenario = mrta_simulation.scenario_spawner:main',
        'manual_task_commander = mrta_simulation.manual_task_commander:main',
        'robot_controller = mrta_simulation.robot_controller_node:main',
        'rl_task_allocator = mrta_simulation.rl_task_allocator:main' 
    ],
},
)