from glob import glob

from setuptools import find_packages, setup

package_name = 'mycobot_280_hardware'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='igor',
    maintainer_email='igor@todo.todo',
    description='myCobot 280 M5 USB driver and RViz destination coordinator.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_monitor = mycobot_280_hardware.robot_monitor_node:main',
            'control_coordinator = mycobot_280_hardware.control_coordinator:main',
            'robot_control = mycobot_280_hardware.robot_control_node:main',
        ],
    },
)
