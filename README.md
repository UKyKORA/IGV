# IGVC
This is the repository for the University of Kentucky IGVC team. The current target hardware is a Raspberry Pi 3 B+ with the Pi ROS image from [Ubiquity](https://downloads.ubiquityrobotics.com/).

## Setup
1. Ensure ROS is installed on the target hardware.
2. `mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src`
3. `git clone https://github.com/UKyKORA/IGV.git igvc`
4. `cd ~/catkin_ws && rosdep install --from-paths src --ignore_src`
5. `cd ~/catkin_ws && catkin_make`
6. `cd ~/catkin_ws/devel && source setup.sh && cd ..`

If the target hardware does not have internet access, use SCP to transfer the repository files to the device.

## Software Architecture

![Architecture Diagram](https://raw.githubusercontent.com/UKyKORA/IGV/7a88e70380e5dd59bf030a0e2a64b666a227b67a/sw_arch_Rev01.png)