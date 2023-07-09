#2023-06-01
#TUK Mechatronics 2018130002 KoGeonHui
#diffrent - test, backmove
#use for - aruco detect pose, distance, us
import time
import numpy as np # ver 1.19.4
#us
#import digitalio
#import keyboard
#motor
#video
from imutils.video import VideoStream
import argparse
import imutils
import cv2 # ver 3.4.18.65
import sys
import json
#uwb
import serial
import struct
#graph
#import matplotlib.pyplot as plt

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
	default="DICT_ARUCO_ORIGINAL",
	help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

detectgray = True
drawaxes = True

if ARUCO_DICT.get(args["type"], None) is None:
	print("[INFO] ArUCo tag of '{}' is not supported".format(
		args["type"]))
	sys.exit(0)

print("[INFO] detecting '{}' tags...".format(args["type"]))
arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT["DICT_5X5_50"])
arucoParams = cv2.aruco.DetectorParameters_create()

with open('/home/ubuntu/ex_code/aruco/camera.json', 'r') as json_file:
	camera_data = json.load(json_file)
dist = np.array(camera_data["dist"])
mtx = np.array(camera_data["mtx"])

vs = VideoStream(src=0).start()
time.sleep(2)

frame = vs.read()

h, w = frame.shape[:2]

newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (h, w), 0, (h, w))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), cv2.CV_32FC1)
x, y, w1, h1 = roi
yh1 = y + h1
xw1 = x + w1

prev_dist = None
prev_time = time.time() + 0.001
values1_buffer = []
values2_buffer = []
prev_values1 = 100
prev_values2 = 100
#timestamps = []
#rev = []
#distuwb = []

# plt.ion()  
# fig, (ax1, ax2) = plt.subplots(2, 1)
# line1, = ax1.plot(timestamps, values1_buffer, label='Value 1')
# line2, = ax2.plot(timestamps, values2_buffer, label='Value 2')
# line3, = ax1.plot(timestamps, distuwb, label='Distuwb')
# line4, = ax2.plot(timestamps, rev, label='rev')
# ax1.set_xlabel('Timestamp')
# ax1.set_ylabel('Value')
# ax1.set_title('Distuwb, Value 1, Value 2 over Time')
# ax1.legend(loc='upper left')
# ax2.set_xlabel('Timestamp')
# ax2.set_ylabel('rev')
# ax2.set_title('rev over Time')
# ax2.legend(loc='upper left')
# plt.tight_layout()

loop = 0

#uwb tx 10pin
ser = serial.Serial('/dev/ttyTHS1',115200,timeout=1)
try:
    while True:
        current_time = time.time()
        # uwb start  
        if ser.in_waiting > 5:
            data1 = ser.read(2)
            data2 = ser.read(2)  
            values1 = struct.unpack('<H',data1)[0]
            values2 = struct.unpack('<H',data2)[0]-50
            print("values1: ", values1, "values2", values2)
            #moving average filter
            # if loop == 0 and (values1 > 500 or values2 > 500):
            #     continue
            # if abs(values1 - prev_values1) < 300 or abs(values2 - prev_values2) < 300:
            #         values1 = prev_values1
            #         values2 = prev_values2
            # timestamps.append(current_time)
            #values1_buffer.append(values1)
            # values2_buffer.append(values2)
            # if len(values1_buffer) > 3:
            #     values1_buffer.pop(0)
            # if len(values2_buffer) > 3:
            #     values2_buffer.pop(0)
            # values1 = sum(values1_buffer) / len(values1_buffer)
            # values2 = sum(values2_buffer) / len(values2_buffer)
            #print("values1: ", values1, "values2", values2)
            #distuwb = (values2 + values1)/2
            # #plt start
            # line1.set_data(timestamps, values1)
            # line2.set_data(timestamps, values2)
            # line3.set_data(timestamps, distuwb)
            # ax1.relim()
            # ax1.autoscale_view()
            # #plt end
            #prev_values1 = values1
            #prev_values2 = values2
            # velocity
            # dist_diff = distuwb - prev_dist
            # time_diff = current_time - prev_time
            # velocity = dist_diff / time_diff
            #print("velocity: ", velocity)       
        time.sleep(0.1)
        # uwb end

        # aruco detect
        frame = vs.read()
        dst1 = cv2.remap
        if detectgray:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            (corners, ids, rejected) = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
        else:
            (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
        if len(corners) > 0:
            if drawaxes:
                # npcorners = np.array(corners)
                # y_coords = npcorners[:, :, 1] 
                # average_y = np.mean(y_coords)   
                # if average_y < 160:
                #         c = 1       # c is speed
                # elif average_y > 160:
                #         c = 0
                for i in range(0, len(ids)):
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, mtx, dist)
                    cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, 0.02) 
                    #x: red y:green z:blue
                    R, _ = cv2.Rodrigues(rvec)
                    z_axis = R[:, 2]
                    x_axis = R[:, 0]
                    y_axis = R[:, 1]
                    # rev == rotate angle
                    rev = np.cross(R[:, 1], R[:, 2])[2]	
                    print("rev: ", rev)
                    
                    # line4.set_data(timestamps, rev)
                    # ax2.relim()
                    # ax2.autoscale_view()
                    
                    distcam = np.linalg.norm(tvec)*178
                    time.sleep(0.5)
                    #print("distcam = {}cm".format(distcam))
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    #    else:
        #    stop()
        #cv2.imshow("Frame", frame)
        # aruco detect end
        end_time = time.time()
        #print("{} time: ".format(loop), end_time - current_time)
        # plt.draw()
        # plt.pause(0.001) 
        loop += 1
finally:
    vs.stop()
#    pca.deinit()
    # ser.close()
    # plt.ioff()
    cv2.destroyAllWindows()

