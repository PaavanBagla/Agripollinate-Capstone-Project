from setuptools import setup
import os
from glob import glob

package_name = 'bee_farm_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('bee_farm_sim/launch/*.launch.py')),
        # Install worlds
        (os.path.join('share', package_name, 'worlds'), glob('bee_farm_sim/worlds/*.world')),
        # Install description
        (os.path.join('share', package_name, 'description'), glob('bee_farm_sim/description/*.xacro')),
    ],


    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paavan',
    maintainer_email='paavan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_filter_node = bee_farm_sim.scripts.scan_filter_node:main',  
            'bee_heatmap_generator = bee_farm_sim.scripts.bee_heatmap_generator:main',
            'bee_controller = bee_farm_sim.scripts.bee_controller:main',    
        ],
    },
)
