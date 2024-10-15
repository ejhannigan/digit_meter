# Project Overview
This is a small project that demonstrates a full pipeline from software to hardware using ROS2. It use machine vision to count fingers detected through a webcam, and the value is displayed by a servo meter. This project uses ROS2 as the glue connecting software to hardware. I have outlined the process for reproducing this project in intricate detail below. 

It is split into three major sections:
1. [Setting up ESP32 to run ROS2](#Setting-up-ESP32-to-run-ROS2)
2. [Setup ROS2 Webcam Publisher](#Create-a-ROS2-webcam-publisher)
3. [Create a ROS2 digit to angle publisher](#Create-a-ROS2-digit-to-angle-publisher)
4. [Appendix: Hardware Setup](#Hardware-Setup)

## Sources:
This project required the use of multiple tutorials from other authors. I want to make sure they get credit right up front!

ROS2 and micro-ros
* [Micro-ROS Official Tutorial](https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/)
* [Mark Smith's Tutorial: Controlling Servos with ESP32 and ROS2](https://medium.com/@markjdsmith/getting-oriented-to-ros2-uros-and-controlling-servos-with-esp32-3b99533ac986)

ROS2 and OpenCV 
* [Automatic Addison Blog: Getting Started with OpenCV in ROS2](https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/)

Mediapipe
* [Enjoy Mechantronics Youtube Chanel: Hand Tracking with MediaPipe and OpenCV](https://www.youtube.com/watch?v=RRBXVu5UE-U)


# Setting up ESP32 to run ROS2:
The hardware setup has its own [appendix](#hardware-setup).

To run ros on a micro controller, I used [micro-ROS](https://micro.ros.org/). I used (this micro-ROS tutorial)[https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/) from the original documentation.

Note: The documentation begins with a suggestion to try [another tutorial](https://medium.com/@SameerT009/connect-esp32-to-ros2-foxy-5f06e0cc64df) for the esp32. This tutorial is not perfect, and I lost a lot of time trying to install esp-idf separately. You DO NOT need to install esp-idf yourself. micro-ROS downloads the esp-idf library from github in the create firmware workspace step. I have included an [appendix](##micro_ros_setup-downloads-esp--idf-for-you) that walks you through the code to prove it to yourself. 

I have copied and pasted the [micro_ros_setup guide](https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/) below and replaced all of the commands with those I used for this project:

```bash
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

These instructions will setup a workspace with a ready-to-use micro-ROS build system.
This build system is in charge of downloading the required cross-compilation tools and building the apps for the
required platforms.

The build system's workflow is a four-step procedure:

* **Create step:** This step is in charge of downloading all the required code repositories and cross-compilation
  toolchains for the specific hardware platform. Among these repositories, it will also download a collection of ready
  to use micro-ROS apps.
* **Configure step:** In this step, the user can select which app is going to be cross-compiled by the toolchain.
  Some other options, such as transport, agent's IP address/port (for UDP transport) or device ID (for serial connections) will be also selected in this step.
* **Build step:** Here is where the cross-compilation takes place and the platform-specific binaries are generated.
* **Flash step:** The binaries generated in the previous step are flashed onto the hardware platform memory,
  in order to allow the execution of the micro-ROS app.
Further information about micro-ROS build system can be found
[here](https://github.com/micro-ROS/micro_ros_setup/tree/dashing/micro_ros_setup).

## Creating a new firmware workspace

Once the build system is installed, let's create a firmware workspace that targets all the required code and tools:

```bash
# Create step
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
```

Once the command is executed, a folder named `firmware` must be present in your workspace.

This step is in charge, among other things, of downloading a set of micro-ROS apps for the specific platform you are
addressing.
In the case of FreeRTOS, these are located at `firmware/freertos_apps/apps`.
Each app is represented by a folder containing the following files:

* `app.c`: This file contains the logic of the application.
* `app-colcon.meta`: This file contains the micro-ROS app specific colcon configuration. Detailed info on how to
  configure the RMW via this file can be found
  [here](/docs/tutorials/advanced/microxrcedds_rmw_configuration/).

For the user to create its custom application, a folder `<my_app>` will need to be registered in this location,
containing the two files just described.

## Creating Custom App

We need to create a custom app to run our servo. I followed a few different tutorials to aid in this process. 
My app is called set_


## Configuring the firmware

The configuration step will set up the main micro-ROS options and select the desired application.
It can be executed with the following command:

```bash
# Configure step
ros2 run micro_ros_setup configure_firmware.sh [APP] [OPTIONS]
```

The options available for this configuration step are:
  - `--transport` or `-t`: `udp`, `serial` or any hardware-specific transport label
  - `--dev` or `-d`: agent string descriptor in a serial-like transport
  - `--ip` or `-i`: agent IP in a network-like transport
  - `--port` or `-p`: agent port in a network-like transport

In this tutorial, we will use a Serial transport (labeled as `serial`) and are going to the custom app from this library called set_servo_angle. You need to manually move the app folder set_servo_angle to the following position: `firmware/freertos_apps/apps/set_servo_angle`. To execute this application with the chosen transport,
run the configuration command above by specifying the `[APP]` and `[OPTIONS]` parameters as below:

```bash
# Configure step with set_servo_angle app and serial transport
ros2 run micro_ros_setup configure_firmware.sh set_servo_angle --transport serial
```

If you want to debug or need a sanity check, you can also use the out-of-the-box ping_pong example. It works perfectly fine with the esp32 as we set it up above. You can follow the [original tutorial](https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/) beginning at the "Configuring the firmware" section. 
## Building the firmware

When the configuring step ends, just build the firmware:

```bash
# Build step
ros2 run micro_ros_setup build_firmware.sh
```

## Flashing the firmware

Flashing the firmware into the platform varies across hardware platforms.
Make sure the esp32 is connected via usb to your computer. 
Run the flash step:

```bash
# Flash step
ros2 run micro_ros_setup flash_firmware.sh
```
## Creating the micro-ROS agent

The micro-ROS app is now ready to be connected to a micro-ROS agent to start talking with the rest of the ROS 2
world.
To do that, let's first of all create a micro-ROS agent:

```bash
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh
```

Now, let's build the agent packages and, when this is done, source the installation:

```bash
# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

## Running the micro-ROS app

At this point, you have both the client and the agent correctly installed.

To give micro-ROS access to the ROS 2 dataspace, you just need to run the agent:

```bash
# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev [device]
```

***TIP:** you can use this command to find your serial device name: `ls /dev/ *`*

In my case, it was /dev/ttyUSB0
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

## Testing the micro-ROS app

At this point, the micro-ROS app is built and flashed and the board is connected to a micro-ROS agent.
We now want to check that everything is working.

We can manually send angles to the int32_subscriber and the motor should move accordingly. 
Open two new command line.

In the first:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Subscribe to micro-ROS int32 subsriber topic
ros2 topic echo /microROS/int32_subscriber
```

In the second:
And now, let's publish a `fake_ping` with ROS 2 from yet another command line:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Send a few fake pings. The servo should move the appropriate angles
ros2 topic pub -1 /microROS/int32_subscriber std_msgs/msg/Int32 'data: 0'
ros2 topic pub -1 /microROS/int32_subscriber std_msgs/msg/Int32 'data: 90'
ros2 topic pub -1 /microROS/int32_subscriber std_msgs/msg/Int32 'data: 180'
```

We should see these messages on the first terminal and the motor should move. 

# Create a ROS2 webcam publisher
Use Webcam to Count Fingers on Your Hand -> Conver to Angle -> Publish to Servo ROS2 Node

# Create a ROS2 digit to angle publisher
Create a digit counter that reads in the webcam image and counts fingers using mediapipe



# Appendix:
## Hardware Setup
The hardware setup is fairly easy.

Materials:
* ESP32
* SG90 servo
* 3xAA battery holder
* breadboard
* jumper cables
* (optional) LEDs for debugging

You need to power the SG90 servo with an outside power source. It draws too much power when commands are sent in quick succession and the board restarts. 
Connect the orange signal cable to GPIO18 (or any [pwm compatible GPIO pin](https://lastminuteengineers.com/esp32-wroom-32-pinout-reference/) but you will have to change the pin in the set_servo_angle app)

Connect the power and ground cables of the SG90 to battery power. 

Connect the ground pin of the ESP32 to the battery ground so everything shares the same ground.

## micro_ros_setup Downloads esp-idf For You
If you replace olimex-stm32-e407 with esp32 in the create_firmware_ws.sh step, the esp-idf library is downloaded for you in the 
```
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
```

[create_firmware_ws.sh](https://github.com/micro-ROS/micro_ros_setup/blob/a146e83297af2d94b103e0ea73b392b5f9b68415/scripts/create_firmware_ws.sh#L116) calls the respective `create.sh` for freertos and esp32. 
![image](https://github.com/user-attachments/assets/42ba5d01-51d4-4d60-81a3-2d0ee29109a6)

And that respective [create.sh](https://github.com/micro-ROS/micro_ros_setup/blob/a146e83297af2d94b103e0ea73b392b5f9b68415/config/freertos/esp32/create.sh#L6C1-L6C79) downloads esp-idf for you:

![image](https://github.com/user-attachments/assets/ec5eb89d-2f6a-4c5c-b998-1b17b53e133d)

