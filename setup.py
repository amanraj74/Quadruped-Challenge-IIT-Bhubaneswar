from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spot_challenge'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aman Jaiswal',
    maintainer_email='aerraj50@gmail.com',
    description='ROS2 Nav2-based autonomous navigation system for quadruped robot with professional obstacle avoidance, path planning, and goal-seeking behaviors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square_path = scripts.square_path:main',
            'circular_path = scripts.circular_path:main',
            'autonomous_navigation = scripts.autonomous_navigation:main',
        ],
    },
)

