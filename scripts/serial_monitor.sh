#!/bin/bash

# Author: Raegan Scheet <cscheet2@unl.edu>
# Date:   2025/19/04
#
# This script starts the serial port of the arduino.
#
# This script was created to avoid bugs relating to
# having having a busy port.
#
# The arduino port is /dev/ttyACM0
#
# To send data to the Ardunio run...
# echo -ne "start\n" > /dev/ttyACM0
# inside of a different terminal
#
# When running this script a new a new directory is
# created called serial_monitor with the data. This
# directory is deleted upon termination.
#
# This file is dependent on the socat package.
#
# Code Adapted from...
# https://unix.stackexchange.com/questions/12359/how-can-i-monitor-serial-port-traffic
# 

# Create New serial_monitor directory
mkdir -p serial_monitor

# Send Serial Port Data
socat /dev/ttyACM0,raw,echo=0 \
  SYSTEM:'tee serial_monitor/in.txt | socat - "PTY,link=/tmp/ttyV0,raw,echo=0,waitslave" | tee serial_monitor/out.txt' &

  # Save socat Process ID
  SOCAT_PID=$!

# Sleep for 2 Seconds to let socat execute
sleep 2

# Clean up socat and serial_monitor when process is exited (Ctrl-C)
trap "echo 'Cleaning up...'; kill $SOCAT_PID; rm -r serial_monitor; exit" SIGINT

# Monitor serial in and out ports and print changes to console
tail -f serial_monitor/in.txt serial_monitor/out.txt
