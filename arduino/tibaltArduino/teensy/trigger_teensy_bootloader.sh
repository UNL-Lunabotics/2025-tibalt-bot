#!/bin/bash
# trigger_teensy_bootloader.sh

# Make sure GPIO is exported
echo 18 > /sys/class/gpio/export 2>/dev/null
echo out > /sys/class/gpio/gpio18/direction

# Pull PROG pin LOW to enter bootloader
echo 0 > /sys/class/gpio/gpio18/value
sleep 0.1
echo 1 > /sys/class/gpio/gpio18/value
