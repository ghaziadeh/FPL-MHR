from final_motor_driver import Motor1, arm_retract ,extend_hook, retract_hook, arm_extend
global motor_left
global motor_right
import time
from time import sleep
import RPi.GPIO as GPIO
motor_right = Motor1(12,16) # pwm was 12
motor_left = Motor1(27,17)# pwm was 27
import csv
import numpy as np
import asyncio
import pandas as pd
from bluetooth import bt_strength, connect_to_bin, scan_bins
global df
import numpy as np
global theta_tot
import os
import sys
import cv2
from pyzbar import pyzbar
# THIS LINE MIGHT CRASH PI BUT WORK ON WINDOWS. CHECK THAT !!!!!!!!!!!!!!
from time import sleep
from datetime import date
from math import cos, sin, pi, floor, tan, atan, pi, sqrt
today = str(date.today())
global encoder_constant
encoder_constant = 500/(pi*7.62*2)
f = open("start_autonomous_flag.txt", "w")
f.write(str(0))
f.close()
x = 5
angle_list = []
global angles
angles = []
y = 0
while y < 360:
    angles.append(y)
    y += 1
fieldnames = ['encoder', 'ult_front_l', 'ult_front_r', 'ult_right', 'ult_left']
while x < 360 + 5:
    angle_list.append(str(x-5)+'_lidar')
    fieldnames.append(angle_list[x-5])
    x += 1
global once
once= 0
global encoder_mem 
global ultra_mem
global lidar_mem
encoder_mem = 0
ultra_mem = 0
lidar_mem = 0

f3 = open("offset.txt","r")
data3 = f3.readline()
f3.close()
offset= float(data3)
if offset > 1:
    offset = 1
global right
global left
right = int(floor(offset*(35)))
left = int(floor(1*(35)))
global p1
global p2
p1 = 7
p2 = 8
GPIO.setup(p1, GPIO.OUT)
GPIO.setup(p2, GPIO.OUT)
# 35% SPEED IS 1 AND 1 
GPIO.output(p1, GPIO.HIGH)
GPIO.output(p2, GPIO.HIGH)
global data
def current_milli_time():
    d = round(time.time() * 1000)
    #print(d)
    return d
def data_fetch(desire='tip', encoder_front=0, encoder_back=0):
    # we need to define lidar array being sent back based on angle increments
    if desire == 'tip':
        global encoder_mem 
        global ultra_mem
        global lidar_mem
        f = open("encoders.txt","r")
        data = f.readlines()
        f.close()
        try:
            top_l = int(data[0])
            top_r = int(data[1])
            encoder = [top_l,top_r]
        except (ValueError, IndexError) as e:
            encoder = encoder_mem
        encoder_mem = encoder
        ultra = 0
        f = open("lidar_server_status.txt","r")
        data = f.readline()
        f.close()
        try:
            server_status= int(data)
            # print(server_status)
        except ValueError:
            #print('fail to check lidar availability')
            server_status= current_milli_time()-10000
        now = current_milli_time()
        lidar_flag = server_status
        #print(now)
        #print(lidar_flag)
        time_diff = abs(now - lidar_flag)
        #print(time_diff)
        global once
        if once ==0:
            time_diff = 0
        #print(once)
        once += 1
        
        if time_diff<40 :
            today = str(date.today())
            # read data from text files or excel
            #returning last row only
            while True:
                try:
                    df = pd.read_csv(r'live_data_{}.csv'.format(today), sep=',')
                    break
                except:
                    try:
                        df = pd.read_csv(r'live_data_{}.csv'.format(today), sep=',')
                        break
                    except:
                        print('HAIL MARY')
                        df = pd.read_csv(r'live_data_{}.csv'.format(today), sep=',')
                        continue
                        
            tip = len(df) - 1
            lidar_l = []
            x = 0
            while x < tip + 1:
                lidar_l.append(float(df['lidar'][x]))
                x += 1
            lidar = {'distances': lidar_l,
                     'angles': angles}
            #print(lidar['angles'][180])
            #print(lidar['distances'][180])
            global ultra_mem
            global lidar_mem 
            encoder_mem = encoder
            ultra_mem= ultra
            lidar_mem = lidar
            f = open("server_status.txt","w")
            f.write(str(0))
            f.close()
            return [encoder[0], ultra, lidar]
        else:
            #print('returning memory data')
            #print(lidar_mem)
            return [encoder_mem[0], ultra_mem, lidar_mem]
    elif desire == 'whole':
        df = pd.read_csv(r'full_server_{}.csv'.format(today), sep=',')
        tip = len(df) - 1
        encoder = df['encoder']
        ultra = df['ultra']
        lidar = []
        lidar = df['lidar']
        return [encoder[0], ultra, lidar]
    elif desire == 'clean_up':
        # the server will subtract whatever is written in encoder offset and write a 0 on that file
        # so next time it will subtract zero.
        f = encoder_front
        b = encoder_back
        print(f'in data_fetch desire of clean, removing {f-b} worth of encoder data')
        file_object = open('encoder_offset.txt', 'w')
        file_object.write(str(f-b))
        file_object.close()
    elif desire == 'transform hyp to opp':
        """
        global theta_tot
        f = encoder_front
        b = encoder_back
        file_object = open('encoder_offset.txt', 'w')
        file_object.write(str(floor(cos(theta_tot*pi/180)*(f - b))))
        file_object.close()
        """
        pass
    else:
        print('invalid data request')
        file_object = open('error_log.txt', 'a')
        file_object.write('\n')
        file_object.write('invalid data request at FPL.py')
        file_object.close()
        
def lidar_singular(angle):
    encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
    angle = int(angle)
    xe = int(angle)
    if lidar['angles'][xe] != angle:
        # locate angle index please
        f = 0
        while f < len(lidar['angles']):
            if angle == lidar['angles'][f]:
                break
            elif angle > lidar['angles'][f]:
                break
            else:
                f += 1
    else:
        f = xe
    if( angle > 270) and (angle < 361):
        tao = 270
        phi = angle - tao
    elif (angle > 0) and (angle < 91):
        tao = 0
        phi = angle - tao
    elif (angle > 90) and (angle < 181):
        tao = 90
        phi = angle - tao
    else:
        tao = 180
        phi = angle - tao
    if tao == 180 or tao == 0:
        tao = 31.25
    else:
        tao = 24.76
    offset = tao/cos(phi * pi / 180)
    # hail mary:
    bogus = 55555
    target = lidar['distances'][f]
    x = 1
    while target == bogus:
        if x %2==0:
            target = lidar['distances'][f+x]
        else:
            target = lidar['distances'][f-x]
        x += 1
    return target - offset


def check_obs_avoiding_ult(rotation, obstacle_address):
    distance = obstacle_address
    theta = (atan((distance+31.25)/24.76)*180/pi)
    right_theta = theta+180
    left_theta = 180 - theta
    left = lidar_singular(left_theta)
    right = lidar_singular(right_theta)
    distances = []
    # 'front_l'
    # 'front_r'
    # 'left'
    #'right'
    if rotation == 'right':
        target = left
        distances.append(target)
        sleep(0.2)
        target = lidar_singular(left_theta)
        distances.append(target)
        sleep(0.2)
        target = lidar_singular(left_theta)
        distances.append(target)
        diff0 = distances[1]-distances[0]
        diff1 = distances[2]-distances[0]
        if diff0 != 0:
            ratio = (diff1 - diff0) / diff0
            print(f'as i rotate right, i am seeing a change in distances with this current rate: {ratio}')
            if (diff1 - diff0) / diff0 > 40/100:
                return True
            else:
                return False
        else:
            return False
    elif rotation == 'left':
        target = right
        distances.append(target)
        sleep(0.2)
        target = lidar_singular(right_theta)
        distances.append(target)
        sleep(0.2)
        target = lidar_singular(right_theta)
        distances.append(target)
    diff0 = distances[1]-distances[0]
    diff1 = distances[2]-distances[0]
    if diff0 != 0:
        ratio = (diff1 - diff0)/diff0
        print(f'as i rotate left, i am seeing a change in distances with this current rate: {ratio}')
        if (diff1 - diff0) / diff0 > 40/100:
            return True
        else:
            return False
    else:
        return False
    

def encoder_to_angle_and_vv(encoder, angle):
    # required: if transforming angle to encoder, must input encoder arg as -1, and vice versa! 
    if angle == -1:
        # define function here to transform encoder increments to total angle rotated (assume constant speed please)
        math_function = 1/(3.17)
        return encoder*math_function
    elif encoder == -1:
        # define function here to transform angle to required encoder value (assume constant speed please)
        math_function = (3.17)
        return angle*math_function
    
    
def lidar_front_checker(angle, threshold):
    # accepted angle inputs: 0,45,90,135,180,225,270,315,360: given angle will return a scan of 45 degrees above and below.
    encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
    x = 0
    distance = threshold
    theta = 90 - (atan((distance+31.25)/24.76)*180/pi)
    left_boundary = angle + theta
    right_boundary = angle - theta
    if left_boundary > 359:
        left_boundary = 359 - left_boundary
    if right_boundary < 0:
        right_boundary = 359 + right_boundary
    #print(lidar['distances'])
    #print(right_boundary)
    #print(left_boundary)
    scan = lidar['distances'][floor(right_boundary):floor(left_boundary)]
    #print(f'scanned values:{scan[0]}')
    bogus = 55555
    x = 0
    end = len(scan)
    too_close = []
    respective_angles = []
    #print(lidar['angles'][0])
    #print(scan[x])
    #print(scan)
    while x < end:
        a = sin(abs(lidar['angles'][x] - 180) * pi / 180) * scan[x]
        perp_dist = sqrt((scan[x] * scan[x]) - (a * a)) - 31.25
        if (perp_dist < threshold) and (scan[x] < bogus):
            too_close.append(scan[x])
            respective_angles.append(lidar['angles'][x])
        x += 1
    if len(too_close) > 0:
        return True, too_close, respective_angles
    else:
        return False, [], []
    
def rotate_right(condition, theta, distance_before):
    if condition == 'obstacle':
        encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        encoder_flag_start = encoder
        motor_left.moveF(left)
        motor_right.moveB(right)
        # define function
        rotation_flag,data,angles = lidar_front_checker(180, distance_before+10)
        while rotation_flag is True:
            rotation_flag,data,angles = lidar_front_checker(180, distance_before+24)
        print('rotated enough to avoid obstacle')
        stop()
        encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        encoder_flag_end = encoder
        angle_rotated = encoder_to_angle_and_vv(encoder_flag_end - encoder_flag_start,-1)
        data_fetch('clean_up', encoder_flag_end, encoder_flag_start)
        return angle_rotated
    elif condition == 'rotate':
        f_encoder = encoder_to_angle_and_vv(-1, theta)
        encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        f_encoder = encoder + f_encoder
        while encoder < f_encoder:
            motor_left.moveF(left)
            motor_right.moveB(right)
            encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        data_fetch('clean_up', encoder, f_encoder)
        stop()
    elif condition == 'null':
        motor_left.moveF(left)
        motor_right.moveB(right)
    return 0

    
def rotate_left(condition, theta, distance_before):
    if condition == 'obstacle':
    
        encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        encoder_flag_start = encoder
        motor_left.moveB(left)
        motor_right.moveF(right)
        # define function
        rotation_flag,data,angles = lidar_front_checker(180, distance_before+10)
        while rotation_flag is True:
            rotation_flag,data,angles = lidar_front_checker(180, distance_before+24)
        print('rotated enough to avoid obstacle')
        stop()
        encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        encoder_flag_end = encoder
        angle_rotated = (encoder_flag_end - encoder_flag_start)/(1)
        data_fetch('clean_up', encoder_flag_end, encoder_flag_start)
        return angle_rotated
    elif condition == 'rotate':
        # ideal constant offset for 50% power was 1.13, now it is 1.11 for expected 100% power performance
        f_encoder = theta
        encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        f_encoder = encoder + f_encoder
        while encoder < f_encoder:
            motor_left.moveB(left)
            motor_right.moveF(right)
            encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        data_fetch('clean_up', encoder, f_encoder)
        stop()
    elif condition == 'null':
        motor_left.moveB(left)
        motor_right.moveF(right)
    # stop using an if function scanning value of encoder
    
    
def stop():
    GPIO.output(p1, GPIO.LOW)
    GPIO.output(p2, GPIO.LOW)
    motor_left.stop()
    motor_right.stop()
#stop()
#sys.exit()
def forward():
    motor_left.moveF(left)
    motor_right.moveF(right)
    
    
def forward_cm(dist, along_what):
    encoder_amount = dist * 5.28
    if along_what == 'null':
        encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        final_encoder = encoder_amount + encoder
        live_encoder = encoder
        while live_encoder < final_encoder:
            forward()
            live_encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        stop()
    elif along_what == 'hypotenuse':
        encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        final_encoder = encoder_amount + encoder
        live_encoder = encoder
        while live_encoder < final_encoder:
            forward()
            live_encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        stop()

        data_fetch('transform hyp to opp', live_encoder, encoder)


def backward():
    motor_left.moveB(left)
    motor_right.moveB(right)


def backward_cm(dist, along_what):
    # 23 is a factor converting cm to increments on encoder data source
    # should be 5.28
    encoder_amount = dist * 1.52
    if along_what == 'null':
        encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        final_encoder = encoder_amount + encoder
        live_encoder = encoder
        while live_encoder < final_encoder:
            backward()
            live_encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        stop()
        stop()



def obstacle_avoidance(phase, tolerance):
    global theta_tot
    # phases:
    # 0--> no obstacle.
    # 1--> obstacle detected and we stopped for 5 seconds just in case, lets check again and if still there rotate
    # 2--> rotated until a clear hypotenuse is made
    # 3--> went along hypotenuse until obstacle to right or left is cleared.
    # 4--> did inverse of phase 2 and ready to move forward again, or we can examine taking phase 5 and 6 on as well
    # 5--> did 2* the inverse of phase 2 and repeated phase 3
    # 6--> repeated original phase 2.
    if phase == 0:
        # check front:
        front = 180
        # check front side to make sure obstacle is 60 cm away
        lidar_flag, lidar_distances, angles = lidar_front_checker(front, tolerance)
        if lidar_flag is True:
            stop()
            print('STOPPING for first spot of obstacle'*20)
            # speaker("move")
            sleep(5)
            return 1
        else:
            return 0
    elif phase == 1:
        front = 180
        lidar_flag, lidar_distances, angles = lidar_front_checker(front, tolerance)
        theta = []
        while lidar_flag is True:
            # this method has two distance checking methods nested in it. in testing phase, check if you want to give up on one
            # we can just use this while true loop and keep checking for a 30 CM distance to move forward.
            #lidar_distances.reverse()
            #angles.reverse()
            obstacle_address = min(lidar_distances)
            x = 0
            while x < len(lidar_distances):
                if lidar_distances[x] == obstacle_address:
                    break
                else:
                    x += 1
            direction = ''
            # if obstacle is on the left side of the image, rotate right, if its on the right, rotate left.
            # lets add some criteria for walls around me to optimize decision making for rotation direction.
            #previous criteria was  if x < len(lidar_distances)/2:
            # which asks which side obstacle is on. now we ask which diagonal is free instead.
            free_diagonal = ''
            min_dist = 180
            if lidar_singular(angle=215)>min_dist:
                # rotating right
                theta.append(rotate_right("obstacle", 0, obstacle_address))
                direction = 'right'
                print('rotating right to avoid obstacle')
            elif lidar_singular(angle=145)>min_dist:
                # rotate left
                theta.append(rotate_left("obstacle", 0, obstacle_address))
                direction = 'left'
                print('rotating left to avoid obstacle')
            else:
                pass
            theta_tot = sum(theta)
            ahead = lidar_singular(angle=181)
            print(f'rotated theta_tot of: {theta_tot}')
            bypass_dist = obstacle_address/cos(theta_tot*pi/180)
            if bypass_dist + 18< ahead:
                forward_cm(bypass_dist + 10, 'hypotenuse')
            else:
                forward_cm(ahead - 20, 'hypotenuse')
            if direction == 'right':
                rotate_left('rotate', theta_tot, obstacle_address)
                print('rotating left after doing r_r and up hyp')
            elif direction == 'left':
                rotate_right('rotate', theta_tot, obstacle_address)
                print('rotating right after doing l_r and up hyp')
            else:
                file_object = open('error_log.txt', 'a')
                file_object.write('\n')
                file_object.write('last part of phase 1 in obstacle avoidance,'
                                  'rotation offset failed to decide direction')
                file_object.close()
            print('obstacle succesfully avoided')
            forward()
            return 0
        forward()
        return 0


        # use one photon to check left back side standard
    """    
    # t_sensor is the sensor that triggers the flag
    obstacle_flag, dist, t_sensor = ultra_sonic_grid_checker('front', 60, sensor='null')
    if obstacle_flag is True:
        ref_dist1 = dist
        stop()
        # speaker("move")
        sleep(5)
    # move a bit closer and try again incase its a human we can scare away:
    if ref_dist1 > 50:
        # move till distance is 50 
        forward_cm(ref_dist1-50)
    # assuming sensor grid consists  
    
        
    obstacle_flag, dist, t_sensor = ultra_sonic_grid_checker('front', 60, sensor='null')
    if obstacle_flag is True:
        ref_dist2 = dist
        stop()
        # speaker("move")
        sleep(3)
    # scan for obstacles one more time!
    obstacle_flag, dist, t_sensor = ultra_sonic_grid_checker('front', 50, sensor='null')
    """


def inside_column():
    if lidar_singular(angle=270) > 200 or lidar_singular(angle=90) > 200:
        return False
    else:
        return True


def align_mhr_with_isle(side):
    if side == 'right':
        #skid right with a 25 degree angle, hence use 115 degree from lidar :)
        phase = 0
        tolerance = 20
        rotate_right('rotate', 25, 0)
        while lidar_singular(angle=255) > 45:
            forward()
            phase = obstacle_avoidance(phase, tolerance)
        stop()
        rotate_left('rotate', 25, 0)
    elif side== 'left':
        # skidding left so lidar looking at 295
        phase = 0
        tolerance = 20
        rotate_left('rotate', 25, 0)
        while lidar_singular(angle=145) > 45:
            forward()
            phase = obstacle_avoidance(phase, tolerance)
        stop()
        rotate_right('rotate', 25, 0)
    else:
        phase = 0
        tolerance = 20
        rotate_left('rotate', 25, 0)
        while lidar_singular(angle=295) > 45:
            forward()
            phase = obstacle_avoidance(phase, tolerance)
        stop()
        rotate_right('rotate', 25, 0)
def bt_signal_enclosure(item):
    if bt_strength(item) == 0:
        return True
    else:
        return False
    
def check_for_stop_or_switch():
    f = open("application_control.txt","r")
    data = f.readline()
    f.close()
    message = str(data)
    print(f'in auto, checking for application commands, i am seeing a : {message}')
    if message == 'stop':
        stop()
        return True
    elif message == 'manual mode':
        return True
    else:
        return False

def read_barcodes(frame):
    barcodes = pyzbar.decode(frame)
    x = -1
    y = -1
    w = -1
    h = -1
    for barcode in barcodes:
        x, y , w, h = barcode.rect
        #1
        barcode_info = barcode.data.decode('utf-8')
        cv2.rectangle(frame, (x, y),(x+w, y+h), (0, 255, 0), 2)
        print(f'x/y: ({x},{y})\nw/h: ({w},{h})')
        #print(y)
        #2
        if (x == 300) or (x == 301) or (x == 302) or (x == 299) or (x == 298):
            return frame, x, y , w, -500
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame, barcode_info, (x + 6, y - 6), font, 2.0, (255, 255, 255), 1)
        #3
    # if all my work is done, let frame = 'collapse'
    return frame, x, y , w, h
def camera_centering():
    #1
    camera = cv2.VideoCapture(0)
    ret, frame = camera.read()
    #2
    print('out loop')
    while ret:
        #print('in ret')
        ret, frame = camera.read()
        frame ,x,y,w,h= read_barcodes(frame)
        if h == -500:
            return 'done'
        cv2.imshow('Barcode/QR code reader', frame)
        #if BLE tag is sensed on the left side (have to add an if statement here)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    camera.release()
    #cv2.destroyAllWindows()
#right = int(floor(offset*(18)))
#left = int(floor(1*(18)))
#motor_left.moveB(left)
#motor_right.moveF(right)
#rotate_right('rotate',170,0)
"""
phase = 0
tolerance =40
while bt_signal_enclosure('6C:E8:5C:5C:D5:87') is False:
    forward()
    if check_for_stop_or_switch() is True:
        stop()
        print('we left the start trip function')
        break
    phase = obstacle_avoidance(phase, tolerance)
    if inside_column() is False:
        # scanned entire column and couldn't find bin!
        # maybe ask for manual supervision ?
        pass
stop()
sleep(1)
sys.exit()
"""

def start_trip(required_travel):
    col = 0
    row = 0
    backward_cm(40,'null')
    rotate_right('rotate',90,0)
    encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
    start_encoder = encoder
    # 259 is 3'6" and 2'6", 31.25 is half robot length, and 76.2 is 2'6" 5 is an offset
    #col_distances = [76.2 + 31.25 + 5, 259.08, 259.08]
    col_distances = [31*22]
    global encoder_constant
    phase = 0
    tolerance = 40
    forward()
    while col < required_travel[0]:
        phase = obstacle_avoidance(phase, tolerance)
        encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        if check_for_stop_or_switch() is True:
            stop()
            return 'stop'
        if encoder-start_encoder > col_distances[col]*5.28:
            start_encoder = encoder
            col += 1

    # enter first row
    rotate_right('rotate', 90, 0)
    forward()
    phase = 0
    column_at_start, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
    while inside_column() is False:
        if check_for_stop_or_switch() is True:
            stop()
            return 'stop'
        phase = obstacle_avoidance(phase, tolerance)
    stop()
    if required_travel[2] > 0:
        # bin is on right side.
        align_mhr_with_isle('right')
    else:
        # bin is on left side.
        align_mhr_with_isle('left')
    #scan_bins()
    # connect_to_bin(required_travel[3])
    # see if you can verify if bin is connected. if not, move forward first.
    # ble_Tag function
    phase = 0
    print('starting BT signal enclosure')
    while bt_signal_enclosure(required_travel[3]) is False:
        forward()
        if check_for_stop_or_switch() is True:
            stop()
            print('we left the start trip function')
            return 'stop'
        phase = obstacle_avoidance(phase, tolerance)
        if inside_column() is False:
            # scanned entire column and couldn't find bin!
            # maybe ask for manual supervision ?
            pass
    stop()
    sleep(1)
    forward_cm(20,'null')
    right = int(floor(offset*(18)))
    left = int(floor(1*(18)))
    GPIO.output(p1, GPIO.HIGH)
    GPIO.output(p2, GPIO.LOW)
    # camera section
    encoder_start, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
    if required_travel[2] > 0:
        # bin is on right side.
        motor_left.moveF(left)
        motor_right.moveB(right)
        while True:
            status = camera_centering()
            break
        stop()
        hyp = lidar_singular(angle = 180)
        encoder_final, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        theta = encoder_to_angle_and_vv(encoder_final-encoder_start, -1)
        rotate_left('rotate', theta, 0)
        forward_cm('null',hyp*cos(theta*pi/180))
        rotate_right('rotate', 90, 0)
        while lidar_singular(angle=180) > 10:
            forward()
        stop()
    else:
        # bin is on left side.
        motor_left.moveB(left)
        motor_right.moveF(right)
        status = camera_centering()
        stop()
        hyp = lidar_singular(angle = 180)
        encoder_final, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        theta = (encoder_final-encoder_start)
        rotate_right('rotate', theta, 0)
        forward_cm('null',hyp*cos(theta*pi/180))
        rotate_left('rotate', 90, 0)
        while lidar_singular(angle=180) > 93:
            forward()
        stop()
    arm_extend() #input the number of steps
    arm_retract() #input the number of step
    retract_hook()
    right = int(floor(offset*(35)))
    left = int(floor(1*(35)))
    GPIO.output(p1, GPIO.HIGH)
    GPIO.output(p2, GPIO.HIGH)
    backward_cm('null',35)
    if required_travel[2] > 0:
        rotate_right('rotate', 90, 0)
    else:
        rotate_left('rotate', 90, 0)
    encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
    phase = 0
    while encoder < column_at_start:
        if check_for_stop_or_switch() is True:
            stop()
            return 'stop'
        phase = obstacle_avoidance(phase, tolerance)
        encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
        forward()
    stop()
    sleep(1)
    forward()
    phase = 0
    while inside_column() is True:
        phase = obstacle_avoidance(phase, tolerance)
    stop()
    rotate_left('rotate', 90, 0)
    col = required_travel[0]
    encoder_f, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
    encoder = encoder_f
    phase = 0
    while encoder< (col_distances[col]*5.28) + encoder_f:
        if check_for_stop_or_switch() is True:
            stop()
            return 'stop'
        phase = obstacle_avoidance(phase, tolerance)
        encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
    stop()
    rotate_right('rotate', 90, 0)
    while lidar_singular(angle=180) > 40:
        forward()
    stop()
    extend_hook()
    arm_extend() #input the number of steps
    retract_hook()
    arm_retract() #input the number of steps
    #done, use ble tag at base to recenter bin for drop off.
    return 'success'
# format: [column,row], and in map[column][row]
# columns : 0, 1 ,2 only
encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
encoder, ultra, lidar = data_fetch(desire='tip', encoder_front=0, encoder_back=0)
print('disconnect now')
list_of_bins = [[1, 5, 9999,'6C:E8:5C:5C:D5:87']]
x = 0
tile_const_35 = 1.535
while True:
    backward()
    sleep(tile_const_35*2)
    stop()
    rotate_right('null',90,0)
    sleep(tile_const_35)
    stop()
    forward()
    sleep(tile_const_35*15)
    stop()
    sleep(5)
    rotate_left('null',90,0)
    sleep(0.79)
    stop()
    forward()
    sleep(tile_const_35*2.5)
    stop()
    rotate_right('null',90,0)
    sleep(0.80)
    stop()
    forward()
    sleep(tile_const_35*10)
    stop()
    sleep(0.1)
    rotate_right('null',90,0)
    sleep(tile_const_35-0.01)
    stop()
    forward()
    sleep(tile_const_35*23)
    stop()
    right = int(floor(offset*(18)))
    left = int(floor(1*(18)))
    rotate_right('null',90,0)
    sleep(tile_const_35+0.08)
    stop()
    rotate_left('null',90,0)
    sleep(0.08)
    stop()
    forward()
    sleep(tile_const_35)
    stop()
    sleep(0.1)
    sys.exit()
    #circuference is :47.85
    # time for 35: 2.37
    # time for 18 is : 4.85
while True:
    f = open("application_control.txt","r")
    data = f.readline()
    f.close()
    message = str(data)
    while message != "automatic mode":
        f = open("application_control.txt","r")
        data1 = f.readline()
        f.close()
        message = str(data1)
        f2 = open("speed.txt","r")
        data2 = f2.readline()
        f2.close()
        try:
            speed = int(data2)
        except ValueError:
            speed = 35
        f3 = open("offset.txt","r")
        data3 = f3.readline()
        f3.close()
        offset= float(data3)
        if offset > 1:
            offset = 1
        #offset = 1
        
        msg=int(speed)
        # print(f'slider value of:{msg}')
        if msg ==0:
            motor_left.stop()
            motor_right.stop()
        right = int(floor(offset*(msg)))
        left = int(floor(1*(msg)))
        
        if message == "on":
            print("LED is ON!")
        
        elif message == "off":
            print("LED is OFF!")
            
        elif message == "forward":
            print("going forward")
            motor_right.moveF(right)
            motor_left.moveF(left)
            
        elif message == "left":
            print("turning left")
            motor_left.moveB(left)
            motor_right.moveF(right)
            
        elif message == "right":
            print("turning right")
            motor_left.moveF(left)
            motor_right.moveB(right)
            
        elif message == "backward":
            print("going backward")
            motor_right.moveB(right)
            motor_left.moveB(left)
        
        elif message == "stop":
            print("stopping")
            motor_left.stop()
            motor_right.stop()
            
        elif message == "pickup bin":
            motor_left.stop()
            motor_right.stop()
            print("picking up bin")
            arm_extend() #input the number of steps
            arm_retract() #input the number of step
            retract_hook()
            
        elif message == "return bin":
            motor_left.stop()
            motor_right.stop()
            print("returning bin")
            extend_hook()
            arm_extend() #input the number of steps
            retract_hook()
            arm_retract() #input the number of steps
            retract_hook()
        elif message == "end of command":
            print("ending command")
            motor_left.stop()
            motor_right.stop()
            
        elif message == "automatic mode":
            print("going into autonomous mode")
            break
            
        elif message == "manual mode":
            print("going into manual mode")
    while message != "manual mode" and message != "stop":
        stop()
        sleep(0.2)
        f = open("application_control.txt","r")
        data1 = f.readline()
        f.close()
        message = str(data1)
        x = 0
        encoder,ultra, lidar= data_fetch(desire='tip',encoder_front = 0,encoder_back = 0)
        print(f'removing {encoder} worth of encoder data to start auto mode')
        data_fetch('clean_up', encoder, 0)
        while x < len(list_of_bins):
            message = start_trip(list_of_bins[0])
            if message == 'stop':
                break

