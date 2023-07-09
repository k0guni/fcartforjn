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
#import matplotlib.pyplot as plt

ser = serial.Serial('/dev/ttyTHS1',115200,timeout=1)
uwblist = []
uwb1 = 0
uwb2 = 0
num = 0
nummain = 0
def UWB():
    global uwb1,uwb2,num,uwblist
    while True:
        if ser.in_waiting >= 5:
            id = ser.read()
            if id == b'\x14':
                num += 1
                data1 = ser.read(2)
                data2 = ser.read(2)
                values1 = struct.unpack('<H',data1)[0]
                values2 = struct.unpack('<H',data2)[0]-50
                uwb1 = values1
                uwb2 = values2
                uwblist.append([uwb1,uwb2])
                #return uwb1, uwb2

#plt.figure()
U = Thread(target=UWB)
U.start()
while True:
    if(uwb1 == 0)and(uwb2 == 0):
        print("pass")
        continue
    u1 = uwblist[-1][0]
    u2 = uwblist[-1][1]
    print("dist1:{},dist2:{}".format(u1,u2))
#    plt.plot(num,u1,'rs--',num,u2,'bs--')        
#    plt.pause(0.01)


