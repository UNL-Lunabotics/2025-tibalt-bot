#!/usr/bin/env python3
import gpiod
import time

CHIP = "gpiochip0"
LINE_OFFSET = 79  # Corresponds to GPIO27, physical pin 13

chip = gpiod.Chip(CHIP)
line = chip.get_line(LINE_OFFSET)

print(f"[INFO] Triggering Teensy bootloader on {CHIP} line {LINE_OFFSET}...")

line.request(consumer="teensy-reset", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])
line.set_value(0)
time.sleep(0.5)
line.set_value(1)
line.release()

print("[INFO] Bootloader trigger complete.")

# import Jetson.GPIO as GPIO
# import time

# teensy_pin = 13

# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(teensy_pin, GPIO.OUT, initial=GPIO.HIGH)

# try:
#     print("[INFO] Triggering Teensy bootloader via GPIO pin 13...")
#     GPIO.output(teensy_pin, GPIO.LOW)
#     time.sleep(0.5)
#     GPIO.output(teensy_pin, GPIO.HIGH)
#     print("[INFO] Bootloader trigger complete.")
# finally:
#     GPIO.cleanup()
    


# import gpiod
# import time

# # GPIO configuration
# CHIP_NAME = "gpiochip0"
# LINE_OFFSET = 79  # Pin 13 on Jetson Orin Nano Dev Kit
# PULSE_DURATION = 0.25

# def trigger_teensy_bootloader(chip_name, line_offset):
#     print(f"[INFO] Triggering Teensy bootloader on {chip_name} line {line_offset}...")

#     # Get the chip and line
#     chip = gpiod.Chip(chip_name)
#     line = chip.get_line(line_offset)

#     # Request line as output and default it HIGH
#     line.request(consumer="teensy_trigger", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])

#     # Pull line LOW
#     print("[INFO] Pulling line LOW")
#     line.set_value(0)
#     time.sleep(PULSE_DURATION)

#     # Set line HIGH again
#     print("[INFO] Releasing line (set HIGH)")
#     line.set_value(1)

#     # Done
#     line.release()
#     chip.close()

#     print("[INFO] Bootloader trigger complete")

# if __name__ == "__main__":
#     trigger_teensy_bootloader(CHIP_NAME, LINE_OFFSET)
