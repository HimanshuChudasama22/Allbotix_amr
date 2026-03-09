import os
from glob import glob
from setuptools import setup

package_name = 'amr_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz') + glob('config/*.rules')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='AMR SLAM package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arduino_bridge = amr_slam.arduino_bridge:main',
            'scan_relay = amr_slam.scan_relay:main',
            'camera_publisher = amr_slam.camera_publisher:main',
            'save_map = amr_slam.save_map:main',
        ],
    },
)
