from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'trayectory_planer_scara'

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
    maintainer='martin',
    maintainer_email='martin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # terminal_name = pkg_name.node_FILE_name
            'trayectory_planer_node = trayectory_planer_scara.trayectory_planner_node:main',
        ],
    },
)
