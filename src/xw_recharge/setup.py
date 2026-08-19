from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xw_recharge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/xw_recharge']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiaowei',
    maintainer_email='dev@xiaowei.local',
    description='Gen2 Laser-Lock Dock auto-recharge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recharge_node = xw_recharge.recharge_node:main',
            'validate_scan = xw_recharge.validate_scan:main',
        ],
    },
)
