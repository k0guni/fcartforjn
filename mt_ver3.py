#!/bin/bash -
import time
import busio

from board import SCL_1, SDA_1
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

i2c = busio.I2C(SCL_1, SDA_1)

pca = PCA9685(i2c, address=0x40)
pca.frequency = 100



pca.channels[2].duty_cycle = 0x0000
pca.channels[2+4].duty_cycle = 0x0000
pca.channels[2+4+4].duty_cycle = 0x0000
pca.channels[2+4+4+4].duty_cycle = 0x0000

print("Forwards slow")
pca.channels[0].duty_cycle = 0x3FFF
pca.channels[1].duty_cycle = 0xFFFF
pca.channels[0+4].duty_cycle = 0x3FFF
pca.channels[1+4].duty_cycle = 0xFFFF
pca.channels[0+4+4].duty_cycle = 0x3FFF
pca.channels[1+4+4].duty_cycle = 0x0000
pca.channels[0+4+4+4].duty_cycle = 0x3FFF
pca.channels[1+4+4+4].duty_cycle = 0x0000
time.sleep(3)

print("Forwards")
pca.channels[0].duty_cycle = 0x1FFF
pca.channels[1].duty_cycle = 0xFFFF
pca.channels[0+4].duty_cycle = 0x1FFF
pca.channels[1+4].duty_cycle = 0xFFFF
pca.channels[0+4+4].duty_cycle = 0x1FFF
pca.channels[1+4+4].duty_cycle = 0x0000
pca.channels[0+4+4+4].duty_cycle = 0x1FFF
pca.channels[1+4+4+4].duty_cycle = 0x0000
time.sleep(3)

print("Backwards")
pca.channels[0].duty_cycle = 0x3FFF
pca.channels[1].duty_cycle = 0x0000
pca.channels[0+4].duty_cycle = 0x3FFF
pca.channels[1+4].duty_cycle = 0x0000
pca.channels[0+4+4].duty_cycle = 0x3FFF
pca.channels[1+4+4].duty_cycle = 0xFFFF
pca.channels[0+4+4+4].duty_cycle = 0x3FFF
pca.channels[1+4+4+4].duty_cycle = 0xFFFF
time.sleep(3)

print("Backwards slow")
pca.channels[0].duty_cycle = 0x1FFF
pca.channels[1].duty_cycle = 0x0000
pca.channels[0+4].duty_cycle = 0x1FFF
pca.channels[1+4].duty_cycle = 0x0000
pca.channels[0+4+4].duty_cycle = 0x1FFF
pca.channels[1+4+4].duty_cycle = 0xFFFF
pca.channels[0+4+4+4].duty_cycle = 0x1FFF
pca.channels[1+4+4+4].duty_cycle = 0xFFFF
time.sleep(3)
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
print("Stop")
pca.channels[0].duty_cycle = 0x0000
pca.channels[1].duty_cycle = 0x0000
pca.channels[0+4].duty_cycle = 0x0000
pca.channels[1+4].duty_cycle = 0x0000
pca.channels[0+4+4].duty_cycle = 0x0000
pca.channels[1+4+4].duty_cycle = 0x0000
pca.channels[0+4+4+4].duty_cycle = 0x0000
pca.channels[1+4+4+4].duty_cycle = 0x0000
time.sleep(3)


pca.deinit()

