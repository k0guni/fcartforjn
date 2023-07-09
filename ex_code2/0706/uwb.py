import serial
import struct
import time

ser = serial.Serial('/dev/ttyTHS1',115200,timeout=1)
print("code start")
while True:
    if ser.in_waiting > 5:
        id = ser.read()
        if id == b'\x14':
            data1 = ser.read(2)
            data2 = ser.read(2)
            values1 = struct.unpack('<H',data1)[0]
            values2 = struct.unpack('<H',data2)[0]-50
            #if (values1 < 3000)and(values1 != 2580)and(values1 != 1300)and(values1 != 2068)and(values2 < 3000)and(values2 != 2580)and(values2 != 1300)and(values2 != 2068)and(values2 != 1812):
                #print("dist1:{},dist2:{}".format(values1,values2))
                #print("dist1:{}".format(values1))
                #print(values1)
            print(values1,values2)
time.sleep(0.1)            