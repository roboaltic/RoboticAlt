from setuptools import setup

package_name = 'diff_drive_l298n'

setup(
    name=package_name,
    version='0.0.1',
    packages=[diff_drive_l298n],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artem',
    description='Differential drive L298N controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'diff_drive_node = diff_drive_l298n.diff_drive_node:main',
        ],
    },
)
