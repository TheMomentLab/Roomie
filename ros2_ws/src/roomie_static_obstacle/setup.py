from setuptools import setup

package_name = 'roomie_static_obstacle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Static obstacle tuner and config for waypoint-based replanning',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tuner_node = roomie_static_obstacle.tuner_node:main',
            'route_manager = roomie_static_obstacle.route_manager_node:main',
        ],
    },
)


