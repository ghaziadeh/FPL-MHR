import time
from time import sleep
from final_motor_driver import Motor1, arm_retract ,extend_hook, retract_hook, arm_extend
import sys
from picamera import PiCamera
import subprocess
print(subprocess.check_output(['ifconfig']))
motor_right = Motor1(12,16)
motor_left = Motor1(27,17)
mem_message = 'asdfgfassadf'
import RPi.GPIO as GPIO
import pandas as pd
import time
from datetime import date
import csv
from math import floor
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.base import MIMEBase
from email.utils import formatdate
from email import encoders
today = str(date.today())
last_data = [-5,-5,-5,-5]
summy_mem = 0
def send_email(file, text):
    send_to = 'abdallah2@mail.usf.edu'
    send_from = 'abdullahatallah2@gmail.com'
    subject = 'MHR encoder data'
    server = smtplib.SMTP('smtp.gmail.com')
    port = '587'
    username = 'abdullahatallah2@gmail.com'
    password = 'Abdullah1234!!'
    isTls = True
    # send_mail(send_from,send_to,subject,text,files,server,port,username,password,isTls)
    Cc = 'recipient'
    msg = MIMEMultipart()
    msg['From'] = send_from
    msg['To'] = send_to
    msg['Cc'] = Cc
    msg['Date'] = formatdate(localtime=True)
    msg['Subject'] = text
    fp = open(file, 'rb')
    part = MIMEBase('application', 'text/csv')
    part.set_payload(fp.read())
    fp.close()
    encoders.encode_base64(part)
    part.add_header('Content-Disposition', 'attachment', filename=file)
    msg.attach(part)
    smtp = smtplib.SMTP('smtp.gmail.com')
    smtp.ehlo()
    smtp.starttls()
    smtp.login(username, password)
    smtp.sendmail(send_from, send_to.split(',') + msg['Cc'].split(','), msg.as_string())
    smtp.quit()
# encoder setup:
# 1 is front left, 2 is front right, 3 is back left, 4 is back right
"""
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
A2_pin = 12
B2_pin = 16
GPIO.setmode(GPIO.BCM)
GPIO.setup(A2_pin, GPIO.IN)
GPIO.setup(B2_pin, GPIO.IN)
outcome2 = [0, -1, 1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0]
last_AB2 = 0b00
counter2 = 0
neg_count2 = 0
itr2 = 0
mem_counter2 = counter2
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
A4_pin = 23
B4_pin = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(A4_pin, GPIO.IN)
GPIO.setup(B4_pin, GPIO.IN)
outcome4 = [0, -1, 1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0]
last_AB4 = 0b00
counter4 = 0
neg_count4 = 0
itr4 = 0
mem_counter4 = counter4

df = pd.DataFrame()
fieldnames = ['encoder_bl', 'encoder_br', 'encoder_fl', 'encoder_fr']

# ,newline=''
with open(r'encoder_calibration_{}.csv'.format(today), 'a') as csv_file:
    csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    csv_writer.writeheader()
"""
# t1 = time.perf_counter()
num = 1
t1 = time.perf_counter()
#camera = PiCamera()
#camera.stop_preview()
#camera.start_preview()
while True:
    
    
    #sleep(7)
    #camera.stop_preview()
    f = open("application_control.txt","r")
    data = f.readline()
    f.close()
    message = str(data)
    
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
    #print(left)
    #print(right)
    #if message == mem_message:
     #   pass
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
        """
        offset -=0.001/4
        f = open(r"offset.txt", "w")
        f.write(str(offset))
        f.close()
        """
        motor_left.moveF(left)
        motor_right.moveB(right)
        
    elif message == "backward":
        print("going backward")
        motor_right.moveB(right)
        motor_left.moveB(left)
        """
        offset +=0.001/4
        f = open(r"offset.txt", "w")
        f.write(str(offset))
        f.close()
        """
    
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
        #extend_hook()
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
        
    elif message == "manual mode":
        print("going into manual mode")
    

    mem_message = message
    """
    ###########################################################################
    one_rotation = 5
    A1 = GPIO.input(A1_pin)
    B1 = GPIO.input(B1_pin)
    current_AB1 = (A1<<1) | B1
    position1 = (last_AB1 << 2) | current_AB1
    last_AB1 = current_AB1
    ###########################################################################
    A2 = GPIO.input(A2_pin)
    B2 = GPIO.input(B2_pin)
    current_AB2 = (A2<<1) | B2
    position2 = (last_AB2 << 2) | current_AB2
    last_AB2 = current_AB2
    ##############################################################################
    A3 = GPIO.input(A3_pin)
    B3 = GPIO.input(B3_pin)
    current_AB3 = (A3<<1) | B3
    position3 = (last_AB3 << 2) | current_AB3
    last_AB3 = current_AB3
    ##############################################################################
    A4 = GPIO.input(A4_pin)
    B4 = GPIO.input(B4_pin)
    current_AB4 = (A4<<1) | B4
    position4 = (last_AB4 << 2) | current_AB4
    last_AB4 = current_AB4
    ##############################################################################
    summy = sum(last_data)
    if (outcome4[position4] != -1) and (outcome3[position3] != -1) and (outcome2[position2] != -1) and (outcome1[position1] != -1):
        if outcome4[position4] == 1:
            counter4 += outcome4[position4]
            if counter4 % one_rotation == 0:
                    itr4 += 1
                    last_data[3]=itr4
                    
        if outcome3[position3] == 1:
            counter3 += outcome3[position3]        
            if counter3 % one_rotation == 0:
                    itr3 += 1
                    last_data[2]=itr3
                    
        if outcome2[position2] == 1:
            counter2 += outcome2[position2]            
            if counter2 % one_rotation == 0:
                    itr2 += 1
                    last_data[1]=itr2
                    
        if outcome1[position1] == 1:
            counter1 += outcome1[position1]            
            if counter1 % one_rotation == 0:
                    itr1 += 1
                    last_data[0]=itr1
                    
    if (summy > 0) and (summy != summy_mem):
        with open(r'live_data_{}.csv'.format(today), 'a') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            info = {
                'encoder_bl': str(itr1),
                'encoder_br': str(itr2),
                'encoder_fl': str(itr3),
                'encoder_fr': str(itr4)
            }
            csv_writer.writerow(info)
    # print(time.perf_counter()-t1)
    # 1 is front left, 2 is front right, 3 is back left, 4 is back right
    # optimization targets are 1 and 2 which is bl and fl
    #print('num of iterations:')
    #print(num)
    num += 1
    t2 = time.perf_counter()
    summy_mem = summy
    timmey = round(t2-t1,1)
    if (timmey % 10 == 0) and itr1 > 1:
        
        df = pd.read_csv(r'live_data_{}.csv'.format(today))
        tip = len(df) - 1
        fl = float(df['encoder_fl'][tip])
        bl = float(df['encoder_bl'][tip])
        fr = float(df['encoder_fr'][tip])
        br = float(df['encoder_br'][tip])
        minimum = min(fl, fr, bl, br)
        maximum = max(fl, fr, bl, br)
        print('analyzing data'*20)
        if (bl > minimum + 6) and (fl > minimum + 6):
            offseter -= 0.01/4
            f = open(r"offset.txt", "w")
            f.write(str(offseter))
            f.close()
            print('slowing right side '*20)
            itr1 = 0
            itr2 = 0
            itr3 = 0
            itr4 = 0
        elif (bl < maximum - 6) and (fl < maximum - 6):
            offseter += 0.01/4
            f = open(r"offset.txt", "w")
            f.write(str(offseter))
            f.close()
            print('speeding right side '*20)
            itr1 = 0
            itr2 = 0
            itr3 = 0
            itr4 = 0
        
        # sys.exit()
    """