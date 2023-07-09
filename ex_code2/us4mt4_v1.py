
#2023-05-26
#TUK Mechatronics 2018130002 KoGeonHui
#refer to - success
#use for - 4 motor with 4 us 


import time
import busio
import digitalio
import board
import keyboard

from board import SCL_1, SDA_1, D10, D9
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL_1, SDA_1)

pca = PCA9685(i2c, address=0x40)
#pca.frquency max 1526 min 24 hz
pca.frequency = 100

pwm_channel = 0
dir_channel = 1
brk_channel = 2

class Sensor():
    def __init__(self, trig_pin, echo_pin):
        self.trig_pin = digitalio.DigitalInOut(trig_pin)
        self.echo_pin = digitalio.DigitalInOut(echo_pin)
        self.trig_pin.direction = digitalio.Direction.OUTPUT
        self.echo_pin.direction = digitalio.Direction.INPUT
    def getDistance(self):
        self.trig_pin.value = False
        self.trig_pin.value = True
        time.sleep(0.00001)        
        self.trig_pin.value = False

        signaloff = 0
        signalon = 0
        timeout = 1
        start_time = time.time()
        while self.echo_pin.value == 0:
            signaloff = time.time()
            if signaloff - start_time > timeout:
                raise RuntimeError("Timeout waiting for signal off")
        while self.echo_pin.value == 1:
            signalon = time.time()
            if signalon - start_time > timeout:
                raise RuntimeError("Timeout waiting for signal on")

        return (signalon - signaloff) * 17000

    def __del__(self):
        self.trig_pin.deinit()
        self.echo_pin.deinit()


def motor(speed, i, c):

    if speed > 100:
        speed = 100
    elif speed < -100:
        speed = -100
    duty_cycle_value = abs(speed//100 * 0xffff//100)
    if speed == 0:
        pca.channels[dir_channel+4*i].duty_cycle = int(0xffff)
        pca.channels[brk_channel+4*i].duty_cycle = int(0xffff)
    elif speed > 0:
        pca.channels[dir_channel+4*i].duty_cycle = int(0xffff)
        pca.channels[brk_channel+4*i].duty_cycle = int(0x0000)
       # pca.channels[pwm_channel+4*i].duty_cycle = int(duty_cycle_value)
        if c == 0:
            pca.channels[pwm_channel+4*i].duty_cycle = 0x2fff
        elif c == 1:
            pca.channels[pwm_channel+4*i].duty_cycle = 0x17ff
    else:
        pca.channels[dir_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[brk_channel+4*i].duty_cycle = int(0x0000)
        #pca.channels[pwm_channel+4*i].duty_cycle = int(duty_cycle_value)
        pca.channels[pwm_channel+4*i].duty_cycle = 0x1fff
        if c == 0:
            pca.channels[pwm_channel+4*i].duty_cycle = 0x2fff
        elif c == 1:
            pca.channels[pwm_channel+4*i].duty_cycle = 0x17ff

def forward(speed):
    motor(-speed,0,0)
    motor(-speed,1,0)
    motor(speed,2,0)
    motor(speed,3,0)
def backward(speed):
    motor(speed,0,0)
    motor(speed,1,0)
    motor(-speed,2,0)
    motor(-speed,3,0)
def right(speed):
    motor(-speed,0,0)
    motor(speed,1,0)
    motor(-1*-speed,2,0)
    motor(-1*speed,3,0)
def left(speed):
    motor(speed,0,0)
    motor(-speed,1,0)
    motor(-1*speed,2,0)
    motor(-1*-speed,3,0)
def ccw(speed):
    motor(-speed,0,1)
    motor(-speed,1,1)
    motor(-1*speed,2,1) 
    motor(-1*speed,3,1)
def cw(speed):
    motor(speed,0,1)
    motor(speed,1,1)
    motor(speed,2,1)
    motor(speed,3,1)
def stop():
    motor(0,0,0)
    motor(0,1,0)
    motor(0,2,0)
    motor(0,3,0)

def main():
    sensors = [
        Sensor(board.D10,board.D9),
        Sensor(board.D25,board.D26),
        Sensor(board.D13,board.D19),
        Sensor(board.D23,board.D22),
        #Sensor(board.D6,board.D12)         
    ]
    while True:
        distances = [sensor.getDistance() for sensor in sensors]
        print(" | ".join("{:.0f}cm".format(distance) for distance in distances))
        if distances[0] < 20:
            if abs(distances[1]-distances[0])>=2.5 :
                while abs(distances[1]-distances[0])>=2.5 :
                    distances = [sensor.getDistance() for sensor in sensors]
                    #print(" | ".join("{:.0f}cm".format(distance) for distance in distances))
                    ccw(5)
                    print("ccw")
                    print(abs(distances[1]-distances[0]))
                    time.sleep(0.015)
            else:
                left(10)
                time.sleep(0.025)
            print("if dis0")
            continue        
        elif distances[3] < 20:
            if abs(distances[1]-distances[0])>=2.5 :
                while abs(distances[3]-distances[2])>=2.5 :
                    distances = [sensor.getDistance() for sensor in sensors]
                    #print(" | ".join("{:.0f}cm".format(distance) for distance in distances))
                    cw(5)
                    print("cw")
                    print(abs(distances[1]-distances[0]))
                    time.sleep(0.015)
            else:
                right(10)
                time.sleep(0.05)
            print("if dis0")
            continue
        else: 
            forward(10)
            print("else forward")
        forward(10)
        print("forward")
        #time.sleep(0.1)
    for sensor in sensors:
        del sensor
if __name__ == "__main__":
    main()
    stop()
    pca.deinit()
