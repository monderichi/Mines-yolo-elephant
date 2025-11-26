from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo_realsense'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='monders',
    maintainer_email='monders@todo.todo',
    description='YOLOv11 object detection with Intel RealSense D455 camera and OBB support',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector = yolo_realsense.yolo_detector:main',
        ],
    },
)
