# Імпорт функції setup для створення Python-пакету
from setuptools import setup

# Робота з файловою системою
import os

# Пошук файлів за шаблоном
from glob import glob


# Назва ROS2-пакету
package_name = 'robot_bringup'


# Основна функція налаштування пакету
setup(

    # Ім’я пакету
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[

        # Реєстрація пакету в ament
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Файл package.xml
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],


    # Залежності Python
    install_requires=['setuptools'],

    # Чи можна архівувати
    zip_safe=True,


    # Дані про автора
    maintainer='artem',
    maintainer_email='artem@todo.todo',

    # Опис пакету
    description='Bringup package for robot (camera + motors)',

    # Ліцензія
    license='Apache License 2.0',

    # Залежності для тестів
    tests_require=['pytest'],


    # Точки входу (ROS2 executables)
    entry_points={

        'console_scripts': [

            # Нода моторів
            # Формат:
            # імʼя = пакет.модуль:функція
            'motor_node = diff_drive_l298n.motor_node:main',

            # Нода камери
            'camera_node = camera_driver.camera_node:main',
        ],
    },
)
