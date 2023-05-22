import time
import RPi.GPIO as GPIO


class Sensor1():
    def __init__(self):
        self.__TRIG1 = 19
        self.__ECHO1 = 21 
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD) 
        GPIO.setup(self.__TRIG1,GPIO.OUT)
        GPIO.setup(self.__ECHO1,GPIO.IN)

    def getDistance(self):
        GPIO.output(self.__TRIG1, GPIO.LOW)
        # TRIG = HIGH
        GPIO.output(self.__TRIG1, True)
        # 0.01ms TRIG = LOW
        time.sleep(0.00001)        
        GPIO.output(self.__TRIG1, False)

        signaloff1=0
        signalon1=0

        while GPIO.input(self.__ECHO1) == 0:
            signaloff1 = time.time()

        while GPIO.input(self.__ECHO1) == 1:
            signalon1 = time.time()

        return (signalon1 - signaloff1) * 17000

    def __del__(self):
        GPIO.cleanup()

class Sensor2():
    def __init__(self):
        self.__TRIG2 = 22 
        self.__ECHO2 = 37 
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD) 
        GPIO.setup(self.__TRIG2,GPIO.OUT)
        GPIO.setup(self.__ECHO2,GPIO.IN)

    def getDistance(self):
        GPIO.output(self.__TRIG2, GPIO.LOW)
        # TRIG = HIGH
        GPIO.output(self.__TRIG2, True)
        # 0.01ms TRIG = LOW
        time.sleep(0.00001)        
        GPIO.output(self.__TRIG2, False)

        signaloff2=0
        signalon2=0
        
        while GPIO.input(self.__ECHO2) == 0:
            signaloff2 = time.time()
        
        while GPIO.input(self.__ECHO2) == 1:
            signalon2 = time.time()
        
        return (signalon2 - signaloff2) * 17000

    def __del__(self):
        GPIO.cleanup()

class Sensor3():
    def __init__(self):
        self.__TRIG3 = 33 
        self.__ECHO3 = 35 
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD) 
        GPIO.setup(self.__TRIG3,GPIO.OUT)
        GPIO.setup(self.__ECHO3,GPIO.IN)

    def getDistance(self):
        GPIO.output(self.__TRIG3, GPIO.LOW)
        # TRIG = HIGH
        GPIO.output(self.__TRIG3, True)
        # 0.01ms TRIG = LOW
        time.sleep(0.00001)        
        GPIO.output(self.__TRIG3, False)

        signaloff3=0
        signalon3=0
        
        while GPIO.input(self.__ECHO3) == 0:
            signaloff3 = time.time()
        
        while GPIO.input(self.__ECHO3) == 1:
            signalon3 = time.time()
        
        return (signalon3 - signaloff3) * 17000

    def __del__(self):
        GPIO.cleanup()

class Sensor4():
    def __init__(self):
        self.__TRIG4 = 16 
        self.__ECHO4 = 15 
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD) 
        GPIO.setup(self.__TRIG4,GPIO.OUT)
        GPIO.setup(self.__ECHO4,GPIO.IN)

    def getDistance(self):
        GPIO.output(self.__TRIG4, GPIO.LOW)
        # TRIG = HIGH
        GPIO.output(self.__TRIG4, True)
        # 0.01ms TRIG = LOW
        time.sleep(0.00001)        
        GPIO.output(self.__TRIG4, False)

        signaloff4=0
        signalon4=0
        
        while GPIO.input(self.__ECHO4) == 0:
            signaloff4 = time.time()
        
        while GPIO.input(self.__ECHO4) == 1:
            signalon4 = time.time()
        
        return (signalon4 - signaloff4) * 17000

    def __del__(self):
        GPIO.cleanup()

class Sensor5():
    def __init__(self):
        self.__TRIG5 = 31 
        self.__ECHO5 = 32 
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD) 
        GPIO.setup(self.__TRIG5,GPIO.OUT)
        GPIO.setup(self.__ECHO5,GPIO.IN)

    def getDistance(self):
        GPIO.output(self.__TRIG5, GPIO.LOW)
        # TRIG = HIGH
        GPIO.output(self.__TRIG5, True)
        # 0.01ms TRIG = LOW
        time.sleep(0.00001)        
        GPIO.output(self.__TRIG5, False)

        signaloff5=0
        signalon5=0
        
        while GPIO.input(self.__ECHO5) == 0:
            signaloff5 = time.time()
        
        while GPIO.input(self.__ECHO5) == 1:
            signalon5 = time.time()
        
        return (signalon5 - signaloff5) * 17000

    def __del__(self):
        GPIO.cleanup()

def main():

    sensor1 = Sensor1()
    sensor2 = Sensor2()
    sensor3 = Sensor3()
    sensor4 = Sensor4()
    sensor5 = Sensor5()

    while True:
        distance1 = sensor1.getDistance()
        distance2 = sensor2.getDistance()
        distance3 = sensor3.getDistance()
        distance4 = sensor4.getDistance()
        distance5 = sensor5.getDistance()
        print("dis1:{:.0f}cm | dis2:{:.0f}cm | dis3:{:.0f}cm | dis4:{:.0f}cm | dis5:{:.0f}cm".format(distance1, distance2, distance3, distance4, distance5))
#        print("{:.0f}cm".format(distance2))

        time.sleep(0.1)
    del sensor1
    del sensor2
    del sensor3
    del sensor4
    del sensor5

if __name__ == '__main__':
    main()
