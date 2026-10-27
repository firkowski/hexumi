# Hexumi

Hexumi is a low-cost, 3D-printed hexapod robot. It has six legs and 18 servo motors (three for each leg). A Raspberry Pi with ROS 2 Jazzy controls the robot. A Pimoroni Servo2040 board with micro-ROS firmware operates the servos.

<img src="/img/HEXUMI_DSSERVO-1.png" height="300" />

## Contents

1. [Description](#1-description)
2. [Repository branches](#2-repository-branches)
3. [Hardware](#3-hardware)
4. [Software components](#4-software-components)
5. [Installation on the Raspberry Pi](#5-installation-on-the-raspberry-pi)
6. [Build the servo driver firmware](#6-build-the-servo-driver-firmware)
7. [Install the firmware on the Servo2040](#7-install-the-firmware-on-the-servo2040)
8. [Operation](#8-operation)
9. [Controller functions](#9-controller-functions)
10. [Simulation on a development computer](#10-simulation-on-a-development-computer)
11. [Troubleshooting](#11-troubleshooting)
12. [Limitations](#12-limitations)
13. [Design goals](#13-design-goals)
14. [References and acknowledgements](#14-references-and-acknowledgements)
15. [Licenses](#15-licenses)

## 1. Description

Hexumi was the subject of a Mechanical Engineering BSc thesis at the Lodz University of Technology. The aim of the project is a walking robot that is cheap and easy to make, repair and change.

You can print all the structural parts on a consumer 3D printer. All the electronic parts are standard parts that you can buy. The cost of one robot can be approximately 300 EUR.

The robot uses a tripod gait. You control the robot with a Nintendo Switch Pro Controller.

## 2. Repository branches

| Branch | Contents | Use |
|---|---|---|
| `main` | All the source code, the CAD files (`CAD/CADs.zip`), the STL files and RViz support. | Use this branch on a development computer. |
| `hexumi-base` | The source code only. RViz does not start. | Use this branch on the Raspberry Pi of the robot. |

## 3. Hardware

The robot has these main parts:

- A Raspberry Pi 4 Model B. This computer runs the ROS 2 nodes.
- A Pimoroni Servo2040 board (RP2040 microcontroller). This board operates the 18 servos.
- 18 servo motors.
- A Nintendo Switch Pro Controller. The controller connects to the Raspberry Pi through Bluetooth.
- The 3D-printed parts. The STL files are in the `STL` folder of the `main` branch.

A USB cable connects the Servo2040 to the Raspberry Pi. The firmware uses this cable for the micro-ROS serial connection.

For the electrical connections, refer to the wiring diagram:

<img src="/img/wiring diagram.png" height="300" />

## 4. Software components

The repository contains these ROS 2 packages (folder `src`):

| Package | Function |
|---|---|
| `hexumi_bringup` | Contains the launch files and the robot parameters (`config/global_params.yaml`). The global parameter server reads the robot model and gives the leg dimensions to the other nodes. |
| `hexumi_teleop_joy` | Changes the controller input into motion commands. |
| `hexumi_kinematics` | Calculates the leg trajectories and the joint angles (inverse kinematics). |
| `hexumi_msgs` | Contains the custom messages for the motion commands. |
| `servo2040_uros` | Contains the messages that the Raspberry Pi and the Servo2040 share. |
| `hexumi_description` | Contains the robot model (URDF/xacro) and the RViz configuration. |

The folder `uros` contains the firmware for the Servo2040. The firmware receives 18 joint angles and moves the servos.

The data flows in this sequence:

```
Pro Controller -> joy_node -> hexumi_teleop_joy -> /teleop/* -> hexumi_kinematics -> /joint_angles -> micro-ROS agent -> Servo2040
```

The kinematics node also sends `/kinematics/joint_states` for RViz.

## 5. Installation on the Raspberry Pi

### 5.1 Requirements

- Ubuntu 24.04 (64-bit) on the Raspberry Pi.
- ROS 2 Jazzy.

### 5.2 Install ROS 2 Jazzy

1. Install Ubuntu Server 24.04 (64-bit) on the Raspberry Pi.
2. Install ROS 2 Jazzy. Follow the instructions in the [ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). The package `ros-jazzy-ros-base` is sufficient.
3. Install the build tools:

   ```bash
   sudo apt install python3-colcon-common-extensions python3-rosdep git
   ```

4. Initialize rosdep. Do this step one time only:

   ```bash
   sudo rosdep init
   rosdep update
   ```

5. Give your user access to the serial port and to the controller:

   ```bash
   sudo usermod -aG dialout,input $USER
   ```

6. Log out and log in again. This makes the group change active.

### 5.3 Install the controller driver

1. Install [joycond](https://github.com/DanielOgorchock/joycond). Follow the instructions in the joycond repository.
2. Pair the Pro Controller with the Raspberry Pi through Bluetooth.
3. Make sure that the controller is available as `/dev/input/js0`:

   ```bash
   ls /dev/input/js0
   ```

### 5.4 Build the Hexumi workspace

1. Make a workspace and clone the `hexumi-base` branch into it:

   ```bash
   mkdir -p ~/hexumi_ws/src
   git clone -b hexumi-base https://github.com/firkowski/hexumi.git ~/hexumi_ws/src/hexumi
   ```

2. Install the dependencies:

   ```bash
   cd ~/hexumi_ws
   source /opt/ros/jazzy/setup.bash
   rosdep install --from-paths src --ignore-src -y
   ```

3. Build the workspace:

   ```bash
   colcon build
   ```

> **NOTE:** colcon does not build the firmware folder `uros`. The file `uros/COLCON_IGNORE` prevents this.

### 5.5 Build the micro-ROS agent

The micro-ROS agent connects the Servo2040 to ROS 2. Build the agent in a different workspace.

1. Make the agent workspace and get `micro_ros_setup`:

   ```bash
   mkdir -p ~/uros_agent_ws/src
   cd ~/uros_agent_ws
   git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
   ```

2. Build `micro_ros_setup`:

   ```bash
   source /opt/ros/jazzy/setup.bash
   rosdep install --from-paths src --ignore-src -y
   colcon build
   source install/local_setup.bash
   ```

3. Build the agent:

   ```bash
   ros2 run micro_ros_setup create_agent_ws.sh
   ros2 run micro_ros_setup build_agent.sh
   ```

> **NOTE:** The agent and the firmware must use the same ROS 2 distribution. A Humble agent does not operate with a Jazzy firmware.

## 6. Build the servo driver firmware

Do these steps on a development computer with Ubuntu 24.04 and ROS 2 Jazzy. You can also do them on the Raspberry Pi, but the build is slow.

### 6.1 Install the Pico SDK

1. Install the compiler and the build tools:

   ```bash
   sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib build-essential
   ```

2. Get the Pico SDK:

   ```bash
   git clone -b 1.5.1 --recurse-submodules https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
   ```

3. Set the SDK path. Add this line to `~/.bashrc`:

   ```bash
   export PICO_SDK_PATH=~/pico-sdk
   ```

4. Open a new terminal. This makes the variable active.

> **NOTE:** The firmware was written for Pico SDK 1.5. Newer SDK versions are not tested.

### 6.2 Build the micro-ROS library

The firmware needs a micro-ROS library that contains the Hexumi messages. You must build this library before you build the firmware. You must build it again each time a message in `servo2040_uros` changes.

1. Clone the `main` branch and get the `micro_ros_setup` submodule:

   ```bash
   git clone https://github.com/firkowski/hexumi.git ~/hexumi
   cd ~/hexumi
   git submodule update --init
   ```

2. Build the tools in the firmware workspace:

   ```bash
   cd ~/hexumi/uros/uros_ws
   source /opt/ros/jazzy/setup.bash
   rosdep install --from-paths src --ignore-src -y
   colcon build
   source install/local_setup.bash
   ```

3. Make the micro-ROS firmware workspace:

   ```bash
   ros2 run micro_ros_setup create_firmware_ws.sh generate_lib
   ```

4. Copy the Hexumi message package into the firmware workspace:

   ```bash
   ros2 run servo2040_uros cp_pkg_to_firmware.sh
   ```

5. Build the library:

   ```bash
   ros2 run micro_ros_setup build_firmware.sh $(pwd)/uros_toolchain.cmake $(pwd)/uros_colcon.meta
   ```

   The library is in `uros/uros_ws/firmware/build`.

### 6.3 Build the firmware

1. Build the firmware:

   ```bash
   cd ~/hexumi/uros
   mkdir -p build
   cd build
   cmake ..
   make -j$(nproc)
   ```

2. Make sure that the file `uros/build/src/hexumi_servo_driver.uf2` is available.

> **NOTE:** If CMake shows the message "micro-ROS headers not found", do the procedure in [6.2](#62-build-the-micro-ros-library) first.

## 7. Install the firmware on the Servo2040

1. Disconnect the Servo2040 from all power sources.
2. Push and hold the BOOT/USER button on the Servo2040.
3. Connect the Servo2040 to the computer with a USB cable.
4. Release the button. The computer shows a USB drive with the name `RPI-RP2`.
5. Copy the file `hexumi_servo_driver.uf2` to the `RPI-RP2` drive.
6. Wait until the drive closes. The Servo2040 then starts the new firmware.

## 8. Operation

> **WARNING:** Keep your hands away from the legs when the servos have power. The servos are strong and can cause injury.

> **CAUTION:** Before the first operation, put the robot on a stand. The legs must not touch the ground. Make sure that each servo moves the correct leg. An incorrect servo connection can damage the robot.

### 8.1 Start the robot

1. Connect the Servo2040 to the Raspberry Pi with the USB cable.
2. Turn on the controller.
3. Open a terminal and start the micro-ROS agent:

   ```bash
   source ~/uros_agent_ws/install/local_setup.bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
   ```

4. Make sure that the agent shows a "session established" message. The firmware tries to connect again until the agent is available.
5. Open a second terminal and start the Hexumi nodes:

   ```bash
   source ~/hexumi_ws/install/setup.bash
   ros2 launch hexumi_bringup test.launch.py
   ```

6. Wait until the kinematics node shows the message "Loaded parameters for 6 legs".
7. Operate the robot with the controller. Refer to [9. Controller functions](#9-controller-functions).

### 8.2 Stop the robot

1. Push B. The robot sits down.
2. Push H (HOME). The servos stop.
3. Push Ctrl+C in each terminal.

> **NOTE:** If you push H while the robot walks, the robot stops the step and sits down from its current position.

### 8.3 Start only the controller nodes

To test the controller without the robot, start only the controller nodes:

```bash
ros2 launch hexumi_bringup teleop.launch.py
```

To see the commands, use:

```bash
ros2 topic echo /teleop/meta_cmd
```

## 9. Controller functions

The robot accepts commands in this sequence: power on, stand up, walk.

| Control | Action | Function |
|---|---|---|
| ZL + ZR | Hold for 3 seconds | Turns on the servos. |
| H (HOME) | Push | Turns off the servos. The robot sits down first. |
| B | Push | Stands up the robot, or sits down the robot. |
| Left stick | Move | Walk. Forward/back moves the robot forward/back. Left/right turns the robot. |
| R + left stick | Hold R and move | Omnidirectional walk. Left/right moves the robot to the side. The robot does not turn. |
| Right stick | Move | Tilts the body. |
| L + right stick | Hold L and move | Moves the body forward, back, left or right. The feet do not move. |
| A | Push | Selects the tripod gait. |
| X | Push | Selects the ripple gait. Refer to [12. Limitations](#12-limitations). |
| Y | Push | Selects the wave gait. Refer to [12. Limitations](#12-limitations). |

The file `src/hexumi_bringup/config/global_params.yaml` sets the maximum speeds and the body limits. For example, `max_lin_velocity` is the maximum walk speed in m/s.

## 10. Simulation on a development computer

You can see the robot motion in RViz without the robot. Use the `main` branch.

1. Install ROS 2 Jazzy Desktop on Ubuntu 24.04.
2. Build the workspace. Use the procedure in [5.4](#54-build-the-hexumi-workspace), but clone the `main` branch.
3. Connect the Pro Controller to the computer.
4. Start the nodes:

   ```bash
   source ~/hexumi_ws/install/setup.bash
   ros2 launch hexumi_bringup test.launch.py
   ```

5. RViz opens and shows the robot model. Operate the model with the controller.

## 11. Troubleshooting

| Problem | Possible cause | Action |
|---|---|---|
| The agent does not show "session established". | The Servo2040 is not on `/dev/ttyACM0`. | Do the command `ls /dev/ttyACM*`. Use the correct device in the agent command. |
| | The agent and the firmware use different ROS 2 distributions. | Build the agent and the firmware library for Jazzy. |
| The agent shows "permission denied". | Your user is not in the group `dialout`. | Do step 5 and step 6 of [5.2](#52-install-ros-2-jazzy). |
| The kinematics node waits for the parameter server. | The global parameter server did not start. | Look for errors in the launch output. Make sure that the robot model is correct. |
| The robot does not respond to the controller. | `joy_node` does not find the controller. | Make sure that `/dev/input/js0` is available. Do the command `ros2 topic echo /joy`. |
| | The servos are off. | Hold ZL + ZR for 3 seconds. Then push B. |
| The teleop node shows "Expected a Pro Controller". | The controller is not a Pro Controller, or joycond does not operate. | Use a Pro Controller with joycond. |
| The servos move to incorrect angles. | The micro-ROS library contains an old message definition. | Do the procedure in [6.2](#62-build-the-micro-ros-library) again. Then build and install the firmware again. |

## 12. Limitations

- Only the tripod gait operates. If you select the wave gait or the ripple gait, the robot uses the tripod gait and shows a warning.
- The IMU function (O button), the foot contact sensors and the current measurement are not available.
- The D-pad body height control is not available.
- If the agent connection stops, the servos keep their last position.
- The servo-to-leg mapping and the calibration of the servo for leg R3 are not verified on the robot. Do the check in the CAUTION in [8. Operation](#8-operation).

## 13. Design goals

- **Low cost.** You can make the robot with a consumer 3D printer and standard electronic parts.
- **Less material.** The printed parts need few supports. Thus, the printer uses less material.
- **Easy repair.** The structure is modular. If a part breaks, you print only that part again. For example, to change the ground contact sensors, you print only the tips of the tibias.
- **Expansion.** The housing has space for more sensors, for example a LIDAR, a camera or an IMU. The robot has a 5 V USB port and can hold a static load of 1.5 kg.
- **Easy use.** The robot has a charge port and other simple interfaces for a new user.

## 14. References and acknowledgements

For the design process and the detailed calculations, refer to the paper "Design and Simulation of a Six-Legged Walking Robot" (2023). The paper is on the author's [ResearchGate profile](https://www.researchgate.net/profile/Maksymilian-Firkowski/research).

The Servo2040 firmware uses code from:

- The [Pimoroni Pico C/C++ library](https://github.com/pimoroni/pimoroni-pico).
- [micro-ROS](https://github.com/micro-ROS/).

## 15. Licenses

- **Source code:** MIT license. Refer to [src/LICENSE](src/LICENSE) and [uros/LICENSE](uros/LICENSE).
- **CAD files, STL files and the wiring diagram:** Creative Commons Attribution 4.0 license. Refer to [CAD/LICENSE](CAD/LICENSE), [STL/LICENSE](STL/LICENSE) and [img/LICENSE](img/LICENSE).
