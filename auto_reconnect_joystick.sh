#!/bin/bash

JOYSTICK_MAC="00:00:00:00:02:4E"

while true; do
    if bluetoothctl info $JOYSTICK_MAC | grep "Connected: yes" > /dev/null; then
        echo "Joystick is already connected"
    else
        echo "Joystick is not connected. Attempting to connect..."
        bluetoothctl connect $JOYSTICK_MAC
    fi
    sleep 10
done
