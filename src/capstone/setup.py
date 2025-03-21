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
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include app files
        (os.path.join('share', package_name, 'app'), glob('app/main.py')),
        (os.path.join('share', package_name, 'app/src'), glob('app/src/*.py')),
        (os.path.join('share', package_name, 'app/ui'), glob('app/ui/*.py')),
        (os.path.join('share', package_name, 'app/ui'), glob('app/ui/*.ui')),
        (os.path.join('share', package_name, 'app/json-styles'), glob('app/json-styles/*.json')),
        # Include QSS files
        (os.path.join('share', package_name, 'app/Qss/scss'), glob('app/Qss/scss/*.scss')),
        # Include icon files
        (os.path.join('share', package_name, 'app/Qss/Icons/feather'), glob('app/Qss/Icons/feather/*.png')),
        (os.path.join('share', package_name, 'app/Qss/Icons/font_awesome/solid'), glob('app/Qss/Icons/font_awesome/solid/*.png')),
        (os.path.join('share', package_name, 'app/Qss/Icons/material_design'), glob('app/Qss/Icons/material_design/*.png')),
        # Include font files
        (os.path.join('share', package_name, 'app/fonts/google-sans-cufonfonts'), glob('app/fonts/google-sans-cufonfonts/*.ttf')),
        # Include other necessary files
        (os.path.join('share', package_name, 'app'), glob('app/_icons_rc.py')),
        (os.path.join('share', package_name, 'app'), glob('app/_icons.qrc')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'sensor_msgs',
        'geometry_msgs',
        # PySide6 dependencies
        'PySide6',
        'numpy',
        'opencv-python',
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
            'device_controller=capstone.device_controller:main',
            # Add entry point for the app
            'gui_app=app.main:main',
        ],
    },
)