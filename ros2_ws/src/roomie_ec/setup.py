from setuptools import find_packages, setup
import os, glob

package_name = 'roomie_ec'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']) + ['scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
        ('share/' + package_name + '/models', ['models/best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Roomie Edge Computing Package',
    license='TODO',
    tests_require=['pytest'],
            entry_points={
            'console_scripts': [
                'button_detector = scripts.button_detector:main',
                'button_controller = scripts.button_controller:main',
                'roomie_ec_node = roomie_ec.roomie_ec_node:main',
                'test_communication_node = scripts.test_communication_node:main',
                'topic_monitor_node = scripts.topic_monitor_node:main',
            ],
        },
) 