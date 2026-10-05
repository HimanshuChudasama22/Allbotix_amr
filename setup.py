from setuptools import setup
from glob import glob

package_name = 'amr_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[

        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name,
            ['package.xml']),

        ('share/' + package_name + '/launch',
            glob('launch/*.py')),

        ('share/' + package_name + '/config',
            glob('config/*.yaml') + glob('config/*.lua') + glob('config/*.rviz') + glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.com',
    description='AMR mapping package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = amr_mapping.teleop_node:main',
            'cmd_vel_bridge = amr_mapping.cmd_vel_bridge:main',
            'wheel_control_node = amr_mapping.wheel_control_node:main',
            'scan_relay_node = amr_mapping.scan_relay:main',
            'map_autosave = amr_mapping.map_autosave:main',
            'a21_ultrasound_node = amr_mapping.a21_ultrasound_node:main',
            'range_filter = amr_mapping.range_filter:main',
            'range_to_scan = amr_mapping.range_to_scan:main',
            # 'orbbec_rgbd_node = amr_mapping.orbbec_rgbd_node:main',
        ],
    },
)
