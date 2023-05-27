import time
import busio
import digitalio
import board

from board import SCL_1, SDA_1, D10, D9
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL_1, SDA_1)

pca = PCA9685(i2c, address=0x40)
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


def motor(speed, i):

    if speed > 100:
        speed = 100
    elif speed < -100:
        speed = -100
    if speed == 0:
        pca.channels[dir_channel+4*i].duty_cycle = int(0xffff)
        pca.channels[brk_channel+4*i].duty_cycle = int(0xffff)
    elif speed > 0:
        pca.channels[dir_channel+4*i].duty_cycle = int(0xffff)
        pca.channels[brk_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[pwm_channel+4*i].duty_cycle = 0x1fff
    else:
        pca.channels[dir_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[brk_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[pwm_channel+4*i].duty_cycle = 0x1fff

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

def main():
    sensors = [
        Sensor(board.D10,board.D9),
        #Sensor(22,37),
        #Sensor(33,35),
        #Sensor(16,15),
        #Sensor(31,32)         
    ]
    while True:
        distance = sensors[0].getDistance()
        if distance <= 30 :
            stop()
            time.sleep(1)
            continue        
        forward(30)
        print(distance)
        #time.sleep(0.1)
    for sensor in sensors:
        del sensor
if __name__ == "__main__":
    main()
    pca.deinit()