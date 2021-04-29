import RPi.GPIO as GPIO
import sys
import pandas as pd
import numpy as np
import time
from time import sleep
from datetime import date
import csv
from adafruit_rplidar import RPLidar
from math import floor
import os
today = str(date.today())
# encoder setup:
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME)
scan_data = [0]*360
angle_list = []
lidar.stop_motor()
lidar.stop()
lidar.disconnect()
lidar = RPLidar(None,PORT_NAME)
"""
# ultrasonic setup:
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

TRIG_L = 11
ECHO_L = 9
TRIG_R = 10
ECHO_R = 22
TRIG_Fl = 25
ECHO_Fl = 18
TRIG_Fr = 8
ECHO_Fr = 7
GPIO.setup(TRIG_L, GPIO.OUT)
GPIO.setup(ECHO_L, GPIO.IN)
GPIO.setup(TRIG_R, GPIO.OUT)
GPIO.setup(ECHO_R, GPIO.IN)
GPIO.setup(TRIG_Fl, GPIO.OUT)
GPIO.setup(ECHO_Fl, GPIO.IN)
GPIO.setup(TRIG_Fr, GPIO.OUT)
GPIO.setup(ECHO_Fr, GPIO.IN)
def ultra_dist(side):
    if side == 'left':
        TRIG = TRIG_L
        ECHO = ECHO_L
    elif side == 'right':
        TRIG = TRIG_R
        ECHO = ECHO_R
    elif side == 'front_l':
        TRIG = TRIG_Fl
        ECHO = ECHO_Fl
    elif side == 'front_r':
        TRIG = TRIG_Fr
        ECHO = ECHO_Fr
    else:
        print('invalid side selection, please choose from (left,right,front)')
    GPIO.output(TRIG,True)
    time.sleep(0.00001)
    GPIO.output(TRIG,False)
    #print('here')
    while GPIO.input(ECHO)==0:
        pulse_start=time.time()
    while GPIO.input(ECHO)==1:
        pulse_end=time.time()
    pulse_duration = pulse_end-pulse_start
    distance=pulse_duration*17150
    distance=round(distance,2)

    return distance
"""
def lidar_distance(angle, lidar_data, lidar_angles):
    x = 0
    while x < len(lidar_angles):
        if lidar_angles[x] == angle:
            # add try except loop in case lidar data is smaller than angle data
            try:
                return lidar_data[x]/10
            except IndexError:
                return 55555
        elif lidar_angles[x] > angle:
            return 55555
        else:
            x += 1
    return 55555

df= pd.DataFrame()
le = 0
"""
from final_motor_driver import Motor1
motor_right = Motor1(3,2)
motor_left = Motor1(27,17)
motor_right.moveF()
motor_left.moveF()
"""
x = 0
fieldnames= []
while x < 360:
    fieldnames.append(str(x)+'_lidar')
    x+=1
x = 0
info = {}
while x < len(fieldnames):
    info[fieldnames[x]] = 55555
    x += 1
#,newline=''
    """
with open(r'live_data_{}.csv'.format(today),'a') as csv_file:
    csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    csv_writer.writeheader()
#print(ultra_dist('front_r'))
print('ultra above')
"""

#print(len(scan_data))
# print(lidar.info)

once = 0
import sys
def current_milli_time():
    return round(time.time() * 1000)
ts = time.perf_counter()
with open(r'live_data_{}.csv'.format(today), 'a') as csv_file:
    csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    print(info)
    csv_writer.writeheader()
print(f'time taken to header fromat: {time.perf_counter()-ts}')
for scan in lidar.iter_scans():
    try:
        scan_data = [0]*360
        angle_list = []
        #print(f'time from last loop to this loop: {time.perf_counter() - t1}')
        for (_, angles, distance) in scan:
            t_angle = floor(angles)
            #print(len(scan_data))
            #print(t_angle)
            scan_data[min([359, t_angle])] = distance
            angle_list.append(min([359, t_angle]))
        angle_list.sort()
        x = 0
        end = len(scan_data)
        while x < end:
            try:
                scan_data.remove(0)
                x += 1
            except ValueError:
                break
        #print(f'time taken for one lidar scan:{time.perf_counter()-ts}')
        #second clean loop
        # print(counter)
        # print(counter)
        # ,newline=''
        # note: if encoder submits new data at a rate that is too low, we will need to use pandas to update ultrasonics independently
        # above in the while true loop, not bounded by the encoder's if. the encoder adds new rows, but ultra updates its current cell.
        if once > 5:
            t1 = time.perf_counter()
            
            #was 6
            f_off = 0
            x = 0
            #print('data:\n')
            #print(angle_list)
            #print(scan_data)
            f = open("lidar_server_status.txt","w")
            f.write(str(current_milli_time()-10000))
            f.close()
            os.remove(r'live_data_{}.csv'.format(today))
            with open(r'live_data_{}.csv'.format(today), 'a') as csv_file:
                csv_writer = csv.DictWriter(csv_file, fieldnames=['lidar'])
                csv_writer.writeheader()
                while x < len(fieldnames):
                    angle = x-0
                    dyn = lidar_distance(angle, scan_data, angle_list)
                    
                    if angle == 270:
                        print(dyn)
                        pass
                    info = {'lidar':dyn}
                    csv_writer.writerow(info)
                    x += 1
            print('sent lidar data to server')
           
            f = open("lidar_server_status.txt","w")
            f.write(str(current_milli_time()))
            f.close()
            #print(f'time taken to DF manip: {time.perf_counter()-t1}')
        #print(info['180_lidar'])
        once += 1
        #print(once)
    except KeyboardInterrupt:
        os.remove(r'live_data_{}.csv'.format(today))
        lidar.stop_motor()
        lidar.stop()
        lidar.disconnect()
        lidar = RPLidar(None,PORT_NAME)
        sys.exit()
    ts = time.perf_counter()
l = ['', '/usr/local/lib/python37.zip', '/usr/local/lib/python3.7', '/usr/local/lib/python3.7/lib-dynload', '/home/pi/.local/lib/python3.7/site-packages', '/usr/local/lib/python3.7/site-packages']

    