from setuptools import setup, find_packages
from glob import glob

package_name = 'vesc_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=['vesc_ros2', 'vesc_ros2.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/vesc.yaml']),
        ('share/' + package_name + '/launch', ['launch/vesc_bringup.launch.py']),
        ('share/vesc_ros2/launch', glob('launch/*.py')),
('share/vesc_ros2/config', glob('config/*.yaml')),

 
   ],
    install_requires=['setuptools', 'pyvesc'],
    zip_safe=True,
    maintainer='khizar',
    maintainer_email='you@example.com',
    description='VESC ROS 2 nodes for command/telemetry',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vesc_command = vesc_ros2.nodes.vesc_command:main',
            'vesc_gui = vesc_ros2.nodes.vesc_gui:main',
 'vesc_true_voltage_logger = vesc_ros2.nodes.vesc_voltage_logger:main',
'vesc_csv_logger=vesc_ros2.vesc_csv_logger:main',
            # you can add telemetry later: 'vesc_telemetry = vesc_ros2.nodes.vesc_telemetry:main',
        ],
    },
)
