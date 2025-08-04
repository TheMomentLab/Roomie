from setuptools import setup
from glob import glob
import os

package_name = 'roomie_ac'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'data'), glob('roomie_ac/data/*')),  
    ],
    install_requires=['setuptools', 'ikpy'],
    zip_safe=True,
    maintainer='mac',
    maintainer_email='jongbob1918@gmail.com',
    description='Arm Controller Action Server',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ac_node = roomie_ac.ac_node:main',
            'yolo_vision_service = roomie_ac.yolo_vision_service:main'
        ],
    },
)
