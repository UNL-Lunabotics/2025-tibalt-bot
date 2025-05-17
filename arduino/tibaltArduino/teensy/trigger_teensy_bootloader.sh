#!/bin/bash

# Example using gpiochip1 line 6 (PIN18)
CHIP=gpiochip0
LINE=80


echo "[INFO] Triggering Teensy PROG pin via GPIO $CHIP line $LINE"

# Pull PROG pin LOW to enter bootloader
gpioset $CHIP $LINE=0
sleep 0.25

# Release pin (HIGH)
gpioset $CHIP $LINE=1

echo "[INFO] Bootloader triggered"

