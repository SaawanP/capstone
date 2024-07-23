from setuptools import setup

package_name = 'capstone'

setup(
    name=package_name,
    version='0.0.0',
    packages=["src/"+package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml', 'launch/host_launch.xml', 'launch/robot_launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Saawan',
    maintainer_email='s463pate@uwaterloo.ca',
    description='Pipe Inspections',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'brain=src.capstone.brain:main',
            'camera=src.capstone.camera:main',
            'defect_ML=src.capstone.defect_ML:main',
            'motor_controller=src.capstone.motor_controller:main',
            'video_feed=src.capstone.video_feed:main',
        ],
    },
)
