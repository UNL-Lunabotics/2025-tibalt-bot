#!/bin/bash

# Path to your compiled firmware
CHIP="gpiochip0"
LINE_OFFSET=79  # GPIO27 = Pin 13 on Jetson Orin Nano
HEX_PATH=".pio/build/teensy41/firmware.hex"
sleep 0.5

echo "[INFO] Triggering Teensy bootloader on $CHIP line $LINE_OFFSET..."

# Trigger pin LOW then HIGH via gpiod
python3 - <<END
import gpiod
import time

chip = gpiod.Chip("$CHIP")
line = chip.get_line($LINE_OFFSET)
line.request(consumer="teensy-reset", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])
line.set_value(0)
time.sleep(0.5)
line.set_value(1)
line.release()
print("[INFO] Bootloader triggered.")
END

sleep 1

# Upload using teensy_loader_cli (must be installed and in PATH)
# -w: wait for device
# -s: reboot after upload
# -mmcu=TEENSY41: target chip
teensy_loader_cli -v -w -s -mmcu=TEENSY41 "$HEX_PATH"
