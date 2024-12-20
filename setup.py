from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'object_detection_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    package_data={
        'object_detection_pkg': ['models/*', 'models/ssd_mobilenet_v2_coco_2018_03_29/*'],
    },
    zip_safe=True,
    maintainer='asli',
    maintainer_email='asli@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = object_detection_pkg.detection_node:main',
            'web_stream = object_detection_pkg.web_stream:main'  
        ],
    },
)