from setuptools import find_packages, setup

package_name = 'roomie_safety_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phj',
    maintainer_email='damuya07@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_monitor_node = roomie_safety_monitor.safety_monitor_node:main',
            'obstacle_simulator = roomie_safety_monitor.obstacle_simulator:main',
        ],
    },
)
