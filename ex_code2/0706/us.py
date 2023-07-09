import time
import numpy as np # ver 1.19.4
from imutils.video import VideoStream
import argparse
import imutils
import cv2 # ver 3.4.18.65
import sys
import json
#uwb
import serial
import struct
from threading import Thread
# import matplotlib
import matplotlib.pyplot as plt

class Sensor():
    def __init__(self, trig_pin, echo_pin):
        self.trig_pin = digitalio.DigitalInOut(trig_pin)
        self.echo_pin = digitalio.DigitalInOut(echo_pin)
        self.trig_pin.direction = digitalio.Direction.OUTPUT
        self.echo_pin.direction = digitalio.Direction.INPUT
    def getDistance(self):
        self.trig_pin.value = False
        self.trig_pin.value = True
        time.sleep(0.00001) # 10us check      
        self.trig_pin.value = False

        signaloff = 0
        signalon = 0
        timeout = 2 # check
        start_time = time.time()
        while self.echo_pin.value == 0:
            signaloff = time.time()
            # if signaloff - start_time > timeout:
            #     raise RuntimeError("Timeout waiting for signal off")
        while self.echo_pin.value == 1:
            signalon = time.time()
            # if signalon - start_time > timeout:
            #     raise RuntimeError("Timeout waiting for signal on")

        return (signalon - signaloff) * 17000

    def __del__(self):
        self.trig_pin.deinit()
        self.echo_pin.deinit()

sensors = [
    Sensor(board.D10,board.D9),
    Sensor(board.D25,board.D26),
    Sensor(board.D13,board.D19),
    Sensor(board.D23,board.D22),
    #Sensor(board.D6,board.D12)         
]

try:      
    while True:
            # us avoid wall
            distances = [sensor.getDistance() for sensor in sensors]
            print(" | ".join("{:.0f}cm".format(distance) for distance in distances))
            for sensor in sensors:
                del sensor        
finally:
    for sensor in sensors:
        del sensor
    print("Done")