from setuptools import setup

package_name = 'diff_drive_l298n'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/diff_drive.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    description='Differential drive L298N controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'diff_drive_node = diff_drive_l298n.diff_drive_node:main',
        ],
    },
)
