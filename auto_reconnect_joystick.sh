#!/bin/bash

JOYSTICK_MAC="00:90:E0:8A:9D:08"

while true; do
    if bluetoothctl info $JOYSTICK_MAC | grep "Connected: yes" > /dev/null; then
        echo "Joystick is already connected"
    else
        echo "Joystick is not connected. Attempting to connect..."
        bluetoothctl connect $JOYSTICK_MAC
    fi
    sleep 10
done
