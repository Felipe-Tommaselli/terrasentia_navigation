#!/bin/sh
sudo cp auto_reconnect_joystick.service /etc/systemd/system/auto_reconnect_joystick.service
sudo cp auto_reconnect_joystick.sh /usr/sbin/auto_reconnect_joystick.sh

sudo systemctl enable auto_reconnect_joystick.service