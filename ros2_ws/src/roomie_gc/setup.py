from setuptools import setup
from glob import glob
import os

package_name = 'roomie_gc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.nodes', f'{package_name}.scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Roomie Guidance Controller (GC) package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roomie_gc_node = roomie_gc.nodes.roomie_gc_node:main',
            'test_servers = roomie_gc.scripts.test_servers:main',
            'tracking_gui = roomie_gc.scripts.tracking_gui:main',
            'speed_limit_controller = roomie_gc.scripts.speed_limit_controller:main',
        ],
    },
)



