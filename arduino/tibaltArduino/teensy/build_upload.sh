#!/bin/bash

echo "[STEP] Triggering Teensy bootloader..."
sudo python3 trigger_teensy_bootloader.py

echo "[STEP] Waiting for Teensy to enter bootloader mode..."

# Wait up to 10 seconds for USB device to appear
for i in {1..10}; do
    if lsusb | grep -q "16C0:0478"; then  # Teensy bootloader VID:PID
        echo "[INFO] Teensy bootloader detected!"
        break
    fi
    sleep 0.5
done

# # Optional timeout check
# if ! lsusb | grep -q "16C0:0478"; then
#     echo "[ERROR] Teensy bootloader not detected. Aborting."
#     exit 1
# fi

echo "[STEP] Cleaning build..."
pio run --target clean

echo "[STEP] Building project..."
pio run

echo "[STEP] Uploading firmware..."
pio run --target upload
