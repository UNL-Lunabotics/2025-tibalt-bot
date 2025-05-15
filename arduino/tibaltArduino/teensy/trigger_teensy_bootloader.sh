#!/bin/bash

# Example using gpiochip4 line 0 (replace with your actual line)
CHIP=gpiochip0
LINE=18

# Pull PROG pin LOW
gpioset $CHIP $LINE=0
sleep 0.1

# Release pin (HIGH)
gpioset $CHIP $LINE=1
