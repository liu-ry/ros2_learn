# ros2_learn

1、安装 ros-humble-autoware-perception-msgs \
  如果顺利的话，直接“sudo apt install -y ros-humble-autoware-perception-msgs” 就可以安装了 \
  如果不顺利，需要具体问题具体分析 \
\
2、终端1 \
source install/setup.bash \
ros2 run perception_msg_demo perception_publisher \
\
3、终端2 \
source install/setup.bash \
ros2 run perception_msg_demo perception_subscriber \
4、终端4 \
rviz2 \
5、再按下图进行可视化：\
![截图 2025-07-06 23-48-12](https://github.com/user-attachments/assets/0da1aa61-8098-4006-9b68-9f0184f06175) \
![截图 2025-07-06 23-50-19](https://github.com/user-attachments/assets/56096b39-1a95-4903-a6df-da11af486b39) \
![截图 2025-07-06 23-50-30](https://github.com/user-attachments/assets/83db586e-5607-4b0c-a55f-5ef9f3cb692d) \
