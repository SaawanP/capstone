from setuptools import setup
from glob import glob
import os

package_name = 'capstone'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools',
                      'rclpy',
                      'std_msgs',
                      'sensor_msgs',
                      'geometry_msgs',
                      ],
    zip_safe=True,
    maintainer='Saawan',
    maintainer_email='s463pate@uwaterloo.ca',
    description='Pipe Inspections',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'backend=capstone.backend:main',
            'brain=capstone.brain:main',
            'camera=capstone.camera:main',
            'motor_controller=capstone.motor_controller:main',
        ],
    },
)
