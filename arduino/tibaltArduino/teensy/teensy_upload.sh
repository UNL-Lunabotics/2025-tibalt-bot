#!/bin/bash

set -e

# Path to your compiled firmware
CHIP="gpiochip0"
LINE_OFFSET=79  # GPIO27 = Pin 13 on Jetson Orin Nano
HEX_PATH=".pio/build/teensy41/firmware.hex"
TEENSY_VID_PID="16C0:0478"

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

# Wait up to 5 seconds for Teensy bootloader to appear
for i in {1..10}; do
    if lsusb | grep -q "$TEENSY_VID_PID"; then
        echo "[INFO] Teensy bootloader detected!"
        break
    fi
    sleep 0.5
done

# Final check
if ! lsusb | grep -q "$TEENSY_VID_PID"; then
    echo "[ERROR] Teensy bootloader not detected. Upload aborted."
    exit 1
fi

echo "[INFO] Uploading firmware to Teensy..."
teensy_loader_cli -v -w -s -mmcu=TEENSY41 "$HEX_PATH"

# Extra delay for flash finalization before system might cut power
sleep 0.5
echo "[INFO] Upload complete. Teensy should now retain firmware across power cycles."