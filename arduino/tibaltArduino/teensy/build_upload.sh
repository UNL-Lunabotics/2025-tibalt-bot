#!/bin/bash
echo "[STEP] Triggering Teensy bootloader..."
sudo ./trigger_teensy_bootloader.sh

echo "[STEP] Waiting for USB to re-enumerate..."
sleep 1

echo "[STEP] Cleaning build..."
pio run --target clean

echo "[STEP] Uploading firmware..."
pio run --target upload
