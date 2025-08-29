# RowBo Boat

[<img src="https://img.youtube.com/vi/XQX0SXHnbyk/0.jpg">](https://youtu.be/XQX0SXHnbyk)

## System

- Ubuntu 22.04
- Jetson Orin Nano
- ROS Humble
- 2 x Piper Robot Arms
- Bluetti AC200L Power System
- Random Canoe

## Setup Packages

    mkdir -p rowboboat_ws/src
    cd rowboboat_ws/src
    git clone --recursive git@github.com:davesarmoury/RowboBoat.git
    cd ..
    source /opt/ros/humble/setup.bash
    rosdep install -i --from-path src -y
    colcon build

## System Setup

    cd rowboboat_ws/src/RowboBoat/rowboboat_bringup/systemd
    ./INSTALL.sh
    sudo reboot
    
