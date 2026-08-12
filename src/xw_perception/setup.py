from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'xw_perception'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/xw_perception']),
    ('share/xw_perception', ['package.xml']),
]

# Install models/ (pose rknn + docs/scripts); skip onnx and unused detect rknn.
models_dir = 'models'
_allow = {
    'yolov8n-pose.rknn',
    'README.md',
    'fetch_model.sh',
    'convert_rknn.sh',
    '.gitignore',
}
if os.path.isdir(models_dir):
    model_files = []
    for name in os.listdir(models_dir):
        path = os.path.join(models_dir, name)
        if os.path.isfile(path) and name in _allow:
            model_files.append(path)
    if model_files:
        data_files.append((os.path.join('share', package_name, 'models'), model_files))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiaowei',
    maintainer_email='dev@xiaowei.local',
    description='Person perception (RKNN pose → tracks / fall)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_stub_node = xw_perception.perception_stub_node:main',
            'person_perception_node = xw_perception.person_perception_node:main',
        ],
    },
)
