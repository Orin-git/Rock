from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xw_sensors'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/xw_sensors']),
    ('share/xw_sensors', ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='xiaowei',
    maintainer_email='dev@xiaowei.local',
    description='Sensor adapters and stubs',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensors_stub_node = xw_sensors.sensors_stub_node:main',
            'depth_topic_bridge = xw_sensors.depth_topic_bridge:main',
        ],
    },
)
