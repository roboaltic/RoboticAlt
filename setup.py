from setuptools import setup

package_name = 'camera_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artem',
    maintainer_email='artem@example.com',
    description='ROS2 node to publish USB camera images',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_driver.camera_node:main'
        ],
    },
)
