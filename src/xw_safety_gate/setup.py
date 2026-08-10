from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'xw_safety_gate'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/xw_safety_gate']),
        ('share/xw_safety_gate', ['package.xml']),
        ('share/xw_safety_gate/config', ['config/safety_gate.yaml']),
]

# install public web assets
if package_name == 'xw_web':
    for dirpath, _, filenames in os.walk('public'):
        if not filenames:
            continue
        install_dir = os.path.join('share', package_name, dirpath)
        files = [os.path.join(dirpath, f) for f in filenames]
        data_files.append((install_dir, files))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiaowei',
    maintainer_email='dev@xiaowei.local',
    description='Laser/ultrasonic safety gate to /cmd_vel',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'safety_gate_node = xw_safety_gate.safety_gate_node:main',
        ],
    },
)
