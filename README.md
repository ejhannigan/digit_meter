Use computer to count your fingers and display the value with a hardware meter. This project uses ROS2 as the glue connecting software to hardware. 

This project requires the use of many outside libraries.

Setting up ESP32 to run ROS2:
To run ros on a micro controller, I used micro-ROS. 
Follow the setup guide in the documentation for freeRTOS: https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/
Note: The documentation begins with a suggestion to try another tutorial for the esp32. This tutorial is not perfect, and I lost a lot of time trying to install esp-idf separately. 
You DO NOT need to install esp-idf yourself. micro-ROS downloads the esp-idf library from github in the create firmware workspace step. 
![image](https://github.com/user-attachments/assets/b2d01c7a-fd94-4363-a3b3-97d1c24b8aaf)

If you replace olimex-stm32-e407 with esp32 in the create_firmware_ws.sh step, the esp-idf library is downloaded for you in the 
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32

create_firmware_ws.sh calls the respective create.sh for freertos and esp32. 
![image](https://github.com/user-attachments/assets/42ba5d01-51d4-4d60-81a3-2d0ee29109a6)

here(https://github.com/micro-ROS/micro_ros_setup/blob/a146e83297af2d94b103e0ea73b392b5f9b68415/scripts/create_firmware_ws.sh#L116)



And https://github.com/micro-ROS/micro_ros_setup/blob/jazzy/config/freertos/esp32/create.sh downloads esp-idf for you:

![image](https://github.com/user-attachments/assets/ec5eb89d-2f6a-4c5c-b998-1b17b53e133d)
here(https://github.com/micro-ROS/micro_ros_setup/blob/a146e83297af2d94b103e0ea73b392b5f9b68415/config/freertos/esp32/create.sh#L6C1-L6C79)



![image](https://github.com/user-attachments/assets/db594c40-62ff-4ede-9234-645f3ee5a25e)






Hardware:
ESP32 dev board
SG90 servo



