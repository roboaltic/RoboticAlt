from setuptools import setup

package_name = 'diff_drive_l298n'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artem',
    maintainer_email='artem@todo.todo',
    description='L298N diff drive controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_drive_node = diff_drive_l298n.motor_node:main',
        ],
    },
)
