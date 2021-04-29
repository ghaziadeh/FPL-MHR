import RPi.GPIO as GPIO
import pandas as pd
import time
import csv
from math import floor
import time
from time import sleep
from datetime import date
today = str(date.today())
last_data = [-5,-5]
summy_mem = 0
num = 0
### top left encoder
A1_pin = 20
B1_pin = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(A1_pin, GPIO.IN)
GPIO.setup(B1_pin, GPIO.IN)
outcome1 = [0, -1, 1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0]
last_AB1 = 0b00
counter1 = 0
neg_count1 = 0
itr1 = 0
mem_counter1 = counter1
###
####top right encoder
A3_pin = 14
B3_pin = 15
GPIO.setmode(GPIO.BCM)
GPIO.setup(A3_pin, GPIO.IN)
GPIO.setup(B3_pin, GPIO.IN)
outcome3 = [0, -1, 1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0]
last_AB3 = 0b00
counter3 = 0
neg_count3 = 0
itr3 = 0
mem_counter3 = counter3
###
### data appension loop
x = 0
fieldnames= ['encoder_fl', 'encoder_fr', 'ult_front_l', 'ult_front_r', 'ult_right', 'ult_left']
while x < 360:
    fieldnames.append(str(x)+'_lidar')
    x+=1
x = 0
info = {}
while x < len(fieldnames):
    info[fieldnames[x]] = 55555
    x += 1
df= pd.DataFrame()
le = 0
#,newline=''
def current_milli_time():
    return round(time.time() * 1000)
f = open("lidar_server_status_internal.txt","w")
f.write(str(1))
f.close()
server_status =1
once = 0
import sys
import os
while True:
    try:
        f = open("encoder_offset.txt","r")
        data = f.readline()
        f.close()
        try:
            remove= int(data)
        except ValueError:
            remove= 0
        if remove != 0:
            itr1 -= (remove-1)
            print(f'to be removed: {remove}')
        ###########################################################################
        one_rotation = 5
        A1 = GPIO.input(A1_pin)
        B1 = GPIO.input(B1_pin)
        current_AB1 = (A1<<1) | B1
        position1 = (last_AB1 << 2) | current_AB1
        last_AB1 = current_AB1
        ##############################################################################
        A3 = GPIO.input(A3_pin)
        B3 = GPIO.input(B3_pin)
        current_AB3 = (A3<<1) | B3
        position3 = (last_AB3 << 2) | current_AB3
        last_AB3 = current_AB3
        summy = sum(last_data)
        if (outcome3[position3] != -1) and (outcome1[position1] != -1):
                        
            if outcome3[position3] == 1:
                counter3 += outcome3[position3]        
                if counter3 % one_rotation == 0:
                        itr3 += 1
                        last_data[1]=itr3
            if outcome1[position1] == 1:
                counter1 += outcome1[position1]            
                if counter1 % one_rotation == 0:
                        itr1 += 1
                        last_data[0]=itr1
   
        if (summy > 0) and (summy != summy_mem):
            print('top left: '+ str(itr1)+'\n'+'top right: '+str(itr3))
            f = open("encoders.txt","w")
            f.write(str(itr1)+'\n'+str(itr3))
            f.close()
        num += 1
        summy_mem = summy
        if remove != 0:
            file_object = open('encoder_offset.txt', 'w')
            file_object.write(str(0))
            file_object.close()
            remove = 0
        
    except KeyboardInterrupt:
        f = open("encoders.txt","w")
        f.write(str(0)+'\n'+str(0))
        f.close()
        sys.exit()
