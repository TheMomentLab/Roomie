from setuptools import setup
import os
import glob

# training 폴더의 모든 파일을 찾기
def get_training_files():
    training_files = []
    for root, dirs, files in os.walk('training'):
        for file in files:
            training_files.append((f'share/roomie_vs/{root}', [os.path.join(root, file)]))
    return training_files

setup(
    name='roomie_vs',
    version='1.0.0',
    packages=['roomie_vs'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/roomie_vs']),
        ('share/roomie_vs', ['package.xml']),
    ] + get_training_files(),
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'pyusb',
        'PyQt6',  # GUI 의존성 추가
    ],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'vs_node = roomie_vs.vs_node:main',
            'vs_gui = roomie_vs.vs_gui_node:main',  # GUI 진입점 추가
        ],
    },
) 