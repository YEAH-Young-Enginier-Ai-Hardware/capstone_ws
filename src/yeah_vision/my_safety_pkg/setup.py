from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_safety_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'models'),
            glob('models/*')
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'audio'),
            glob('audio/*')
        ),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='kim',
    maintainer_email='kim@todo.todo',
    description='Safety monitoring package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fall_detection_node = my_safety_pkg.fall_detection_node:main',
            'helmet_detection_node = my_safety_pkg.helmet_detection_node:main',
            'odom_ws_bridge = my_safety_pkg.odom_ws_bridge:main',
            'speaker_warning_node = my_safety_pkg.speaker_warning_node:main',
        ],
    },
)
