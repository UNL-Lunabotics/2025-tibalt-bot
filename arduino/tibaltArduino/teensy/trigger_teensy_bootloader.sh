#!/bin/bash

# Example using gpiochip4 line 0 (replace with your actual line)
CHIP=gpiochip1
LINE=4

echo "[INFO] Triggering Teensy PROG pin via GPIO $CHIP line $LINE"

# Pull PROG pin LOW
gpioset $CHIP $LINE=0
sleep 0.1

# Release pin (HIGH)
gpioset $CHIP $LINE=1

echo "[INFO] Bootloader triggered"