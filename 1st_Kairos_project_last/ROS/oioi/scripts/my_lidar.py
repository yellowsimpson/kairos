#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import signal
import sys
# import rospy

def radar_open():
    # rospy.init_node('ydlidar_ros_test', anonymous=True)
    def radar_high():
        GPIO.setmode(GPIO.BCM)
        time.sleep(0.1)
        GPIO.setup(20, GPIO.OUT)
        GPIO.output(20, GPIO.HIGH)

    radar_high()
    time.sleep(0.05)

def radar_close():
    def radar_low():
        GPIO.setmode(GPIO.BCM)
        time.sleep(0.1)
        GPIO.setup(20, GPIO.OUT)
        GPIO.output(20, GPIO.LOW)

    radar_low()
    time.sleep(0.05)

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    radar_close()
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    radar_open()
    print('Press Ctrl+C to exit')
    while True:
        time.sleep(1)
