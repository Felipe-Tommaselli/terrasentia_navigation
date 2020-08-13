#!/bin/sh
sudo cp roscore.service /etc/systemd/system/roscore.service
sudo cp env.sh /etc/ros/env.sh
sudo cp roslaunch.service /etc/systemd/system/roslaunch.service
sudo cp roslaunch /usr/sbin/roslaunch

sudo systemctl enable roscore.service
sudo systemctl enable roslaunch.service
sudo chmod +x /usr/sbin/roslaunch