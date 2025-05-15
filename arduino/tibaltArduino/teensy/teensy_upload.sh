#!/bin/bash

# Path to your compiled firmware
HEX_PATH=".pio/build/teensy41/firmware.hex"

# Upload using teensy_loader_cli (must be installed and in PATH)
# -w: wait for device
# -s: reboot after upload
# -mmcu=TEENSY41: target chip
teensy_loader_cli -v -w -s -mmcu=TEENSY41 "$HEX_PATH"
