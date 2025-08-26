from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'roomie_vs'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinhyuk',
    maintainer_email='jinhyuk@todo.todo', # TODO: 실제 이메일로 변경 필요
    description='Roomie 로봇을 위한 비전 시스템 패키지입니다.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = roomie_vs.vision_node:main',
            'test_vs_client = roomie_vs.test_vs_client:main',
            'button_ocr_tool = roomie_vs.scripts.button_ocr_tool:main',
            'ocr_test_tool = roomie_vs.scripts.ocr_test_tool:main',
            'test_person_tracker = roomie_vs.scripts.test_person_tracker:main',
        ],
    },
)