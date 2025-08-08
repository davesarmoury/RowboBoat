#!/bin/bash

## Setup CAN
sudo cp rowboboat_can_activate.sh /usr/local/bin/
sudo chmod +x /usr/local/bin/rowboboat_can_activate.sh

sudo cp rowboboat_can.service /etc/systemd/system
sudo systemctl enable rowboboat_can.service

## Start ROS
sudo cp rowboboat.sh /usr/local/bin/
sudo chmod +x /usr/local/bin/rowboboat.sh

sudo cp rowboboat.service /etc/systemd/system
sudo systemctl enable rowboboat.service

