#!/bin/bash

# Example using gpiochip1 line 4 (PIN16)
CHIP=gpiochip1
LINE=4

echo "[INFO] Triggering Teensy PROG pin via GPIO $CHIP line $LINE"

# Pull PROG pin LOW
gpioset $CHIP $LINE=0
sleep 0.1

# Release pin (HIGH)
gpioset $CHIP $LINE=1

echo "[INFO] Bootloader triggered"