from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'roomie_vs'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    scripts=[
        'scripts/test_vs_client.py',
    ],
    python_requires=">=3.6",
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'geometry_msgs',
        'std_msgs',
        'cv_bridge',
        'numpy',
        'opencv-python',
        'roomie_msgs',
        'pyusb',
        'PyQt6',  # GUI 의존성 추가
    ],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'vs_node = roomie_vs.vs_node:main',
            'vs_gui = roomie_vs.vs_gui_node:main',  # GUI 진입점 추가
            'vs_test_client = roomie_vs.test_vs_client:main',
        ],
    },
) 