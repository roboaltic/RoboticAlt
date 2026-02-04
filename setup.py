from setuptools import setup
import os
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),  # <-- вот этот момент
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artem',
    maintainer_email='artem@todo.todo',
    description='Bringup package for robot (camera + motors)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
