Eldar Shamsudinov(KSM-22)
#  Комп’ютери та мережа - (Ubuntu + ROS2)

##  Про проєкт

У межах навчальної практики було розгорнуто локальну мережу з трьох ПК, встановлено Ubuntu Desktop та ROS2, а також налаштовано автоматизацію запуску необхідних інструментів.

Проєкт демонструє базові навички:

* адміністрування Linux
* мережевого налаштування
* встановлення ROS2
* роботи з ROS2 workspace
* автоматизації запуску застосунків

---

##  Мета

* Налаштувати локальну мережу між ПК
* Встановити Ubuntu Desktop
* Встановити та перевірити ROS2
* Створити ROS2 workspace
* Налаштувати автозапуск ROS2
* Реалізувати запуск застосунків одним кліком

---

##  Середовище

| Компонент    | Значення               |
| ------------ | ---------------------- |
| ОС           | Ubuntu 22.04 / 24.04   |
| ROS2         | Humble / Jazzy         |
| Мережа       | Ethernet (через світч) |
| Кількість ПК | 3                      |

---

##  Налаштування мережі

### Перевірка підключення
ip a
ping <ip-іншого-ПК>

### Виявлена проблема

Один ПК не був у мережі через несправний/неправильний порт світча.

### Рішення

Перепідключення кабелю в інший порт.

 Після цього всі ПК почали бачити один одного.

---

##  Встановлення Ubuntu

Після інсталяції виконано оновлення системи:
sudo apt update
sudo apt upgrade

---

##  Встановлення ROS2

### Підключення середовища
source /opt/ros/humble/setup.bash

### Перевірка роботи

У різних терміналах:
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener

 Вузли успішно обмінюються повідомленнями.

---

##  ROS2 Workspace

### Створення
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build

### Підключення workspace
source ~/ros2_ws/install/setup.bash

---

##  Автопідключення ROS2

У файл ~/.bashrc додано:
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

Тепер ROS2 підключається автоматично при відкритті термінала.

---

##  Скрипт швидкого запуску

Створено скрипт start_app.sh, який:

* відкриває 2 термінали
* запускає ROS2-вузол
* відкриває браузер на потрібному сайті
* запускає програму вебкамери

### Приклад
#!/bin/bash

gnome-terminal &
gnome-terminal &

gnome-terminal -- bash -c "ros2 run demo_nodes_cpp talker; exec bash" &

firefox https://example.com &
cheese &

---

##  Desktop Launcher

Створено .desktop файл для запуску з робочого столу.

### Важливо

Надати права:
chmod +x start_app.sh
chmod +x start_app.desktop

### Типова проблема

 broken desktop file

Причини:

* немає прав на виконання
* неправильний шлях у Exec
* файл створено в Windows-форматі

---

##  Troubleshooting

###  ros2: command not found
source /opt/ros/humble/setup.bash

---

###  colcon build: command not found
sudo apt install python3-colcon-common-extensions

---

###  dpkg was interrupted
sudo dpkg --configure -a

---

###  ПК не бачить мережу

Перевірити:

* кабель
* порт світча
* IP-адресу (`ip a`)
* пінг між ПК

---

##  Результати

У ході практики:

*  налаштовано локальну мережу
*  встановлено Ubuntu
*  встановлено ROS2
*  створено workspace
*  налаштовано автозапуск
*  реалізовано запуск одним кліком

---

##  Висновок

Було отримано практичні навички роботи з Linux, мережами та ROS2. Проведено діагностику типових проблем та реалізовано автоматизацію запуску робочого середовища.

Проєкт готовий як базова платформа для подальших робототехнічних розробок.

---

##  Корисні джерела

* https://docs.ros.org
* https://ubuntu.com/tutorials
* https://colcon.readthedocs.io
* https://wiki.ros.org
* https://help.ubuntu.com
