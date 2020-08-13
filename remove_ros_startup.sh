#!/bin/sh
sudo rm /etc/systemd/system/roscore.service
sudo rm /etc/ros/env.sh
sudo rm /etc/systemd/system/roslaunch.service
sudo rm /usr/sbin/roslaunch

sudo systemctl disable roscore.service
sudo systemctl disable roslaunch.service