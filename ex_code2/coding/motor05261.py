import time
import busio
import keyboard

from board import SCL_1, SDA_1
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL_1, SDA_1)

pca = PCA9685(i2c, address=0x40)
pca.frequency = 100

pwm_channel = 0
dir_channel = 1
brk_channel = 2


def motor(speed, i):

    if speed > 100:
        speed = 100
    elif speed < -100:
        speed = -100
    if speed == 0:
        pca.channels[brk_channel+4*i].duty_cycle = int(0xffff)
        pca.channels[dir_channel+4*i].duty_cycle = int(0xffff)
    elif speed > 0:
        speed_scaled = int((speed/100.0)*65535)
        speed_pwm = '0x{:04X}'.format(speed_scaled)
        pca.channels[brk_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[dir_channel+4*i].duty_cycle = int(0xffff)
        pca.channels[pwm_channel+4*i].duty_cycle = int(0x18ff)
    else:
        speed_scaled = int((-speed/100.0)*65535)
        speed_pwm = '0x{:04X}'.format(speed_scaled)
        pca.channels[brk_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[dir_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[pwm_channel+4*i].duty_cycle = int(0x18ff)

def forward(speed):
    motor(-speed,0)
    motor(-speed,1)
    motor(speed,2)
    motor(speed,3)
def backward(speed):
    motor(speed,0)
    motor(speed,1)
    motor(-speed,2)
    motor(-speed,3)
def right(speed):
    motor(-speed,0)
    motor(speed,1)
    motor(-1*-speed,2)
    motor(-1*speed,3)
def left(speed):
    motor(speed,0)
    motor(-speed,1)
    motor(-1*speed,2)
    motor(-1*-speed,3)
def cw(speed):
    motor(-speed,0)
    motor(-speed,1)
    motor(-1*speed,2) 
    motor(-1*speed,3)
def ccw(speed):
    motor(speed,0)
    motor(speed,1)
    motor(speed,2)
    motor(speed,3)
def stop():
    motor(0,0)
    motor(0,1)
    motor(0,2)
    motor(0,3)

while 1:
    stop()
    print("act:")
    act = input()

    if act == 'f':
        start_time = time.time()
        while True:
            forward(5)
            time.sleep(0.1)
            if keyboard.is_pressed('q'):
                break
        end_time = time.time()
        elapsed_time = end_time - start_time
        print(elapsed_time)
    elif act == 'b':
        backward(5)
        time.sleep(1)
        stop()
    elif act == 'r':
        right(5)
        time.sleep(1)
        stop()
    elif act == 'l':
        left(5)
        time.sleep(1)
        stop()
    elif act == 'c':
        cw(5)
        time.sleep(1)
        stop()
    elif act == 'w':
        ccw(5)
        time.sleep(1)
        stop()
    elif stop == 's':
        stop()
        time.sleep(1)

pca.deinit()
