#!/bin/bash
bash trigger_teensy_bootloader.sh
sleep 1
pio run --target clean
pio run --target upload
