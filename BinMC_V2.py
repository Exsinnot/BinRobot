import threading
import time
import cv2 as cv
import pandas as pd
import json
import csv
import os
from flask_cors import CORS
from flask import Flask, request, jsonify ,Response
from flask_socketio import SocketIO, emit
import asyncio
from gpiozero import LED,PWMLED,DistanceSensor
from ultralytics import YOLO
import json
import speech_recognition as sr
import websockets
import socket
import numpy as np
import subprocess
import math
from adafruit_servokit import ServoKit
import random
import board
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219
import busio
import adafruit_vl53l0x
from digitalio import DigitalInOut
from adafruit_vl53l0x import VL53L0X
import pygame
import mediapipe as mp
from pydub import AudioSegment
import glob
from io import BytesIO
import RPi.GPIO as GPIO
import time
import board
import busio
from digitalio import DigitalInOut
from adafruit_vl53l0x import VL53L0X
from mpu6050 import mpu6050
from datetime import datetime

sensor = mpu6050(0x68,bus=0)
i2c_bus = busio.I2C(1, 0)
pin_br = 27  
pin_fr = 22  
pin_bl = 17  
pin_fl = 4   

GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.output(20,0)
GPIO.output(21,0)
time.sleep(0.1)
GPIO.output(20,1)
GPIO.output(21,1)


vl53 = VL53L0X(i2c=i2c_bus,address=0x29)

vl53.measurement_timing_budget = 50000
vl53.signal_rate_limit = 0.1

kit = ServoKit(i2c=i2c_bus,channels=8)
ina219 = INA219(i2c_bus,0x41)
ina219.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
ina219.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
ina219.bus_voltage_range = BusVoltageRange.RANGE_16V
sensor1 = DistanceSensor(echo=11, trigger=9,max_distance=8) #IN
sensor2 = DistanceSensor(echo=24, trigger=23,max_distance=8) #F
sensor3 = DistanceSensor(echo=14, trigger=15,max_distance=8) #R
sensor4 = DistanceSensor(echo=6, trigger=13,max_distance=8) #F
GPIO.setmode(GPIO.BCM)
GPIO.setup(25, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(pin_br, GPIO.OUT)
GPIO.setup(pin_fr, GPIO.OUT)
GPIO.setup(pin_bl, GPIO.OUT)
GPIO.setup(pin_fl, GPIO.OUT)
GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP) 

pwm_br = GPIO.PWM(pin_br, 100)  
pwm_fr = GPIO.PWM(pin_fr, 100)
pwm_bl = GPIO.PWM(pin_bl, 100)
pwm_fl = GPIO.PWM(pin_fl, 100)

pwm_br.start(0)
pwm_fr.start(0)
pwm_bl.start(0)
pwm_fl.start(0)

ip = ""
ip_server = None
ports = 8001
CONNECTION = set()
camera_x = 90
camera_y = 120
kit.servo[0].angle = 120 #X
kit.servo[4].angle = 0 #Y
modear = True
datebatterylist = []
distanceFix = 0
modebackstap = True
batterypersenwatt = None
internet_con = False
gohome = 0
keyword = ""
distanceSensor_data = [0,0,0,0]
def wattperperseninmin(filename = ""):
    global datebatterylist
    filename_parts = filename.split("/")
    df = pd.read_csv(f'/home/user/BinRobot/data/{filename_parts[2]}-{filename_parts[1]}-{filename_parts[0]}.csv', header=None)
    df['time'] = pd.to_datetime(df[0], format="%d-%m-%y %H:%M:%S")
    df['battery_percent_diff'] = df[1].diff().abs()
    df['watt_per_percent'] = df[2] / df['battery_percent_diff']
    df_filtered = df.dropna()
    df_filtered = df_filtered[df_filtered['battery_percent_diff'] != 0]
    average_watt_per_percent = df_filtered['watt_per_percent'].mean()
    return average_watt_per_percent
with open("/home/user/BinRobot/data/Setting.json", mode='r', encoding='utf-8') as file:
    jsondata = json.load(file)
    distanceFix = int(jsondata['stopdistance'])
    gohome = int(jsondata['gohome'])
    keyword = jsondata['name']

# model = YOLO("yolov8n.pt")
bus_voltage = ina219.bus_voltage  
battery = int((bus_voltage - 10.5) / 0.015)
app = Flask(__name__)
CORS(app)
net = cv.dnn.readNet('yolov4-tiny.weights', 'yolov4-tiny.cfg')
with open('coco.names', 'r') as f:
    classes = f.read().strip().split('\n')
frame = None
cap = None
frameweb = None
coms = {
    "Goto":None,
    "Last": None,
    "Point":None,
    "Mode": 3,
    "SPDL": 0.8,
    "SPDR": 0.8,
    "Break" : True
}

timereset = True
def W(Speed):
    pwm_br.ChangeDutyCycle(0)              
    pwm_fr.ChangeDutyCycle(Speed+2)      
    pwm_bl.ChangeDutyCycle(0)              
    pwm_fl.ChangeDutyCycle(Speed)   
def S(Speed):
    pwm_br.ChangeDutyCycle(Speed)    
    pwm_fr.ChangeDutyCycle(0)   
    pwm_bl.ChangeDutyCycle(Speed)   
    pwm_fl.ChangeDutyCycle(0)  
def A(Speed,Degee = None):
    pwm_br.ChangeDutyCycle(0)    
    pwm_fr.ChangeDutyCycle(Speed)   
    pwm_bl.ChangeDutyCycle(Speed)   
    pwm_fl.ChangeDutyCycle(0)  
def D(Speed,Degee = None):
    pwm_br.ChangeDutyCycle(Speed)    
    pwm_fr.ChangeDutyCycle(0)   
    pwm_bl.ChangeDutyCycle(0)   
    pwm_fl.ChangeDutyCycle(Speed)  
def Stop():
    pwm_br.ChangeDutyCycle(0)    
    pwm_fr.ChangeDutyCycle(0)   
    pwm_bl.ChangeDutyCycle(0)   
    pwm_fl.ChangeDutyCycle(0)  
    

def playsound(name):
    folder_path = f'/home/user/BinRobot/sound/{name}/*.mp3'
    file_names = glob.glob(folder_path)
    file_name = file_names[random.randint(0,len(file_names)-1)]
    mp3_audio = AudioSegment.from_mp3(file_name)
    mp3_audio = mp3_audio - 10  

    wav_io = BytesIO()
    mp3_audio.export(wav_io, format="wav")

    wav_io.seek(0)
    pygame.mixer.init()
    pygame.mixer.music.load(wav_io, 'wav')

    pygame.mixer.music.play()


    while pygame.mixer.music.get_busy():
        continue


yaw = 0
prev_time = time.time()
num_calibration_samples = 200
gyro_bias = {'x': 0, 'y': 0, 'z': 0}
print("Calibrating gyroscope...")

for i in range(num_calibration_samples):
    gyro_data = sensor.get_gyro_data()
    gyro_bias['x'] += gyro_data['x']
    gyro_bias['y'] += gyro_data['y']
    gyro_bias['z'] += gyro_data['z']
    time.sleep(0.01)

gyro_bias['x'] /= num_calibration_samples
gyro_bias['y'] /= num_calibration_samples
gyro_bias['z'] /= num_calibration_samples

print("Calibration complete.")

# websocket
async def handler(websocket):
    global coms,camera_y,camera_x
    CONNECTION.add(websocket)
    if len(CONNECTION) <= 1:
        print("Server >>> client connected.")
        try:
            coms["Mode"] = 1
            print("Sever >>> Mode 1")
            event = {
                "state" : True,
                "speak" : "Welcome! Yon can working in this page.",
             }
            coms["Goto"] = None
            await websocket.send(json.dumps(event))
            async for message in websocket:
                data = json.loads(message)
                print(data["status"])
                coms["SPDL"] = 0.8
                coms["SPDR"] = 0.8
                if data["status"] == "up":
                    camera_y += 0.5
                    if camera_y > 180:
                        camera_y = 180
                    kit.servo[0].angle = int(camera_y)
                elif data["status"] == "down":
                    camera_y -= 0.5
                    if camera_y < 0:
                        camera_y = 0
                    kit.servo[0].angle = int(camera_y)
                elif data["status"] == "left":
                    camera_x += 0.5
                    if camera_x > 180:
                        camera_x = 180
                    kit.servo[4].angle = int(camera_x)
                elif data["status"] == "right":
                    camera_x -= 0.5
                    if camera_x < 0:
                        camera_x = 0
                    kit.servo[4].angle = int(camera_x)
                elif data["status"] == "reset":
                    camera_y = 90
                    kit.servo[0].angle = int(camera_y)
                elif data["status"] == "w":
                    #print("w")
                    W(50)
                elif data["status"] == "s":
                    #print("s")
                    S(50)
                elif data["status"] == "a":
                    #print("a")
                    A(30)
                elif data["status"] == "d":
                    #print("d")
                    D(30)
                elif data["status"] == "off":
                    Stop()
                        
            await websocket.wait_closed()
        except websockets.exceptions.ConnectionClosedError as e:
            print(f"Server >>> Connection closed with error: {e}")
        except asyncio.exceptions.IncompleteReadError as e:
            print(f"Server >>> Incomplete read error: {e}")
        except Exception as e:
            print(f"Server >>> Unexpected error: {e}")
        finally:
            coms["Mode"] = 0
            print("Server >>> Mode 0")
            CONNECTION.remove(websocket)
    else:
        try:
            event = {
                "state" : False,
                "speak" : "It one person working in page.",
            }
            await websocket.send(json.dumps(event))
        except websockets.exceptions.ConnectionClosedError as e:
            print(f"Server >>> Connection closed with error: {e}")
        except asyncio.exceptions.IncompleteReadError as e:
            print(f"Server >>> Incomplete read error: {e}")
        except Exception as e:
            print(f"Server >>> Unexpected error: {e}")
        finally:
            CONNECTION.remove(websocket)
            print("Server >>> Connection removed. Current connections:", len(CONNECTION))

async def main():
    try:
        print("Server >>> Websocket start.")
        async with websockets.serve(handler, ip, ports):
            await asyncio.Future()
    except:
        print("Server >>> Websocket shutdown.")

def get_sensor():
    global distanceSensor_data
    while True:
        distanceSensor_data[0] = sensor1.distance * 100
        distanceSensor_data[1] = sensor2.distance * 100
        distanceSensor_data[2] = sensor3.distance * 100
        distanceSensor_data[3] = sensor4.distance * 100
        time.sleep(0.1)

def get_bounding_box(hand_landmarks, frame_width, frame_height):
    min_x = min([landmark.x for landmark in hand_landmarks.landmark]) * frame_width
    min_y = min([landmark.y for landmark in hand_landmarks.landmark]) * frame_height
    max_x = max([landmark.x for landmark in hand_landmarks.landmark]) * frame_width
    max_y = max([landmark.y for landmark in hand_landmarks.landmark]) * frame_height
    return (min_x, min_y), (max_x, max_y)
def automode():
    global frame, yaw, kit, coms, mp_hands,ip_server
    mp_hands = mp.solutions.hands
    with mp_hands.Hands(
            static_image_mode=True,
            max_num_hands=2,
            min_detection_confidence=0.5
        ) as hands:
        while True:
            if frame is None or coms["Mode"] != 0 or ip_server == None:
                time.sleep(1)
                continue
            frame = cv.flip(frame, 1) 
            rgb_frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            result = hands.process(rgb_frame)
            if result.multi_hand_landmarks:
                for hand_landmarks in result.multi_hand_landmarks:
                    (min_x, min_y), (max_x, max_y) = get_bounding_box(hand_landmarks, 1280, 720)
                    box_area = (max_x - min_x) * (max_y - min_y)
                    frame_area = 1280 * 720
                    hand_percentage = (box_area / frame_area) * 100
                    if hand_percentage > 10:
                        Stop()
                        openbin("on")
                        time.sleep(10)
                        openbin("off")
                        continue
            if distanceSensor_data[1] < 65 or distanceSensor_data[2] < 20 or distanceSensor_data[3] < 20:
                if distanceSensor_data[1]  < 65:
                    Stop()
                    S(30)
                    time.sleep(1.2)
                    Stop()
                    W(30)
                    time.sleep(0.3)
                    Stop()
                    if distanceSensor_data[2]  > 100:
                        D(30)
                        time.sleep(0.8)
                    elif distanceSensor_data[3]  > 100:
                        A(30)
                        time.sleep(0.8)
                    elif distanceSensor_data[2]  < distanceSensor_data[3] :
                        A(30)
                        time.sleep(0.8)
                    elif  distanceSensor_data[3]  < distanceSensor_data[2] :
                        D(30)
                        time.sleep(0.8)
                    else:
                        D(30)
                        time.sleep(1)
                    Stop()
                else:
                    Stop()
                    S(30)
                    time.sleep(0.4)
                    Stop()
                    if distanceSensor_data[2]  > 100:
                        D(30)
                        time.sleep(0.8)
                    elif distanceSensor_data[3]  > 100:
                        A(30)
                        time.sleep(0.8)
                    elif distanceSensor_data[2] < distanceSensor_data[3]:
                        A(30)
                        time.sleep(0.8)
                    elif distanceSensor_data[3] < distanceSensor_data[2] :
                        D(30)
                        time.sleep(0.8)
                    else:
                        D(30)
                        time.sleep(1)
                    Stop()
            else:
                W(35)

def interruptIR():
    global modebackstap
    while True:
        if coms["Mode"] != 4 and not modebackstap:
            time.sleep(1)
            continue
        if GPIO.input(25) and modebackstap:
            modebackstap = False  
            Stop()
        time.sleep(0.05)

def Battery():
    global yaw,modebackstap,kit,camera_y,battery
    model = YOLO("best.pt")
    time_person = time.time()
    lastcom = ""
    find_st = False
    goto_st = 0
    time_reset = time.time()
    while True:
        if frame is None or coms["Mode"] != 4:
            time.sleep(1)
            continue
        if battery < 10:
            Stop()
            playsound("stuck")
            coms['Mode'] = 5
            continue
        try:
            if modebackstap:
                if distanceSensor_data[1]  < 65 or distanceSensor_data[2] < 20 or distanceSensor_data[3] < 20:
                    if distanceSensor_data[1]  < 65:
                        Stop()
                        S(30)
                        time.sleep(2.5)
                        Stop()
                        if distanceSensor_data[2]  > 100:
                            D(30)
                            time.sleep(1.4)
                        elif distanceSensor_data[3]  > 100:
                            A(30)
                            time.sleep(1.4)
                        elif distanceSensor_data[2]  < distanceSensor_data[3] :
                            A(30)
                            time.sleep(1.4)
                        elif distanceSensor_data[3]  < distanceSensor_data[2] :
                            D(30)
                            time.sleep(1.4)
                        else:
                            D(30)
                            time.sleep(1.4)
                        time_person = time.time()
                        find_st = False
                        Stop()
                        W(30)
                        time.time(2.5)
                        Stop()
                        lastcom == ""
                if time.time()-time_person > 30 and not find_st:
                    W(30)
                    yaw = 0
                    time_person = time.time()
                elif time.time()-time_person > 5 and not find_st:
                    if lastcom == "D":
                        D(30)
                    elif lastcom == "A":
                        A(30)
                    elif yaw <= 0:
                        D(30)
                        lastcom = "D"
                    elif yaw > 0:
                        A(30)
                        lastcom = "A"
                    time.sleep(0.5)
                    Stop()
                    time.sleep(1) 
                if not find_st:
                    results = model.predict(frame,conf=0.6)
                    for x in results[0].boxes:
                        xy = x.xyxy[0].tolist()
                        time_person = time.time()
                        posx = int(((xy[2]-xy[0])/2)+xy[0])
                        posy = int(((xy[3]-xy[1])/2)+xy[1])
                        if posy > 400:
                            camera_y -= 1
                            kit.servo[0].angle = int(camera_y)
                        elif posy < 300:
                            camera_y += 1
                            kit.servo[0].angle = int(camera_y)
                        if posx > 540 and posx < 740:
                            find_st = True
                            goto_st = time.time()
                            yaw = 0
                        elif posx < 540:
                            A(30)
                            time.sleep(0.1)
                            Stop()
                        elif posx > 740:
                            D(30)
                            time.sleep(0.1)
                            Stop()
                if find_st:
                    if time.time() - goto_st < 10:
                        if yaw < -2:
                            D(20)
                            time.sleep(0.1)
                            Stop()
                        elif yaw > 2:
                            A(20)
                            time.sleep(0.1)
                            Stop()
                        else:
                            W(20)
                    else:
                        time_person = time.time()
                        find_st = False
                    
            else:
                time_reset = time.time()
                Stop()
                yaw = 0
                while yaw < 180:
                    D(20)
                    time.sleep(0.05)
                Stop()
                while not GPIO.input(25):
                    if not GPIO.input(25):
                        for i in range(10):
                            W(20)
                            time.sleep(0.1)
                            if GPIO.input(25):
                                time_reset = time.time()
                                Stop()
                                break
                    if not GPIO.input(25):
                        for i in range(10):
                            S(20)
                            time.sleep(0.1)
                            if GPIO.input(25):
                                time_reset = time.time()
                                Stop()
                                break
                    if time.time() - time_reset > 60:
                        break
                while True:
                    if not GPIO.input(25):
                        while not GPIO.input(25):
                            if not GPIO.input(25):
                                for i in range(5):
                                    D(25)
                                    time.sleep(0.1)
                                    if GPIO.input(25):
                                        time_reset = time.time()
                                        Stop()
                                        break
                            if not GPIO.input(25):
                                for i in range(10):
                                    A(25)
                                    time.sleep(0.1)
                                    if GPIO.input(25):
                                        time_reset = time.time()
                                        Stop()
                                        break
                    elif distanceSensor_data[1] < 50:
                        Stop()
                        if distanceSensor_data[1] < 50:
                            yaw = 0
                            S(30)
                            time.sleep(1.2)
                            Stop()
                            while yaw < 180:
                                D(20)
                                time.sleep(0.05)
                            Stop()
                    else:
                        time_reset = time.time()
                        S(25)
                    if time.time() - time_reset > 60:
                        break
                    time.sleep(0.1)
        except:
            print("error")

def follow():
    global frame, yaw, kit, coms, mp_hands,camera_y,distanceFix,frameweb
    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose()
    time_person = time.time()
    lastcom = ""
    fist_time = False
    while True:
        try:
            if frame is None or coms["Mode"] != 2:
                fist_time = False
                time.sleep(1)
                continue
            frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            results = pose.process(frame_rgb)
            if results.pose_landmarks:
                landmarks = results.pose_landmarks.landmark

                left_hand_y = landmarks[mp_pose.PoseLandmark.LEFT_WRIST].y
                right_hand_y = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].y
                head = landmarks[mp_pose.PoseLandmark.NOSE].y
                head_y = landmarks[mp_pose.PoseLandmark.NOSE].y * 720
                head_x = landmarks[mp_pose.PoseLandmark.NOSE].x * 1280
                if left_hand_y < head or right_hand_y < head:
                    time_person = time.time()
                    if head_y < 300:
                        camera_y += 3
                        kit.servo[0].angle = int(camera_y)
                    min_x = min([lm.x for lm in landmarks])
                    max_x = max([lm.x for lm in landmarks])
                    min_y = min([lm.y for lm in landmarks])
                    max_y = max([lm.y for lm in landmarks])

                    frame_height = 720
                    frame_width = 1280
                    bbox_width = (max_x - min_x) * frame_width
                    bbox_height = (max_y - min_y) * frame_height
                    bbox_area = bbox_width * bbox_height
                    frame_area = frame_width * frame_height
                    person_area_percentage = (bbox_area / frame_area) * 100
                    if person_area_percentage > 50:
                        if distanceSensor_data[1]  < 80:
                            yaw = 0
                            time.sleep(0.1)
                            distance1 = 120 if vl53.range/10 >= 120 else vl53.range/10
                            while distance1 > distanceFix + 10:
                                if yaw < -10:
                                    D(20)
                                elif yaw > 10:
                                    A(20)
                                else:
                                    W(20)
                                time.sleep(0.1)
                                distance1 = 120 if vl53.range/10 >= 120 else vl53.range/10
                            Stop()  
                            lastcom = ""
                            print("person off")
                            camera_y = 120
                            kit.servo[0].angle = int(camera_y)
                            openbin("on")
                            time.sleep(10)
                            openbin("off")
                            Stop()
                            coms["Mode"] = 0
                            time_person = time.time()
                    if head_x > 540 and head_x < 740:
                        if not fist_time:
                            fist_time = True
                            Stop()
                            playsound("iseeu")
                        yaw = 0
                        W(30)
                    elif head_x < 540:
                        A(30)
                        time.sleep(0.1)
                        Stop()
                    elif head_x > 740:
                        D(30)
                        time.sleep(0.1)
                        Stop()
            if distanceSensor_data[1]  < 65 or distanceSensor_data[2] < 20 or distanceSensor_data[3] < 20:
                if distanceSensor_data[1]  < 65:
                    Stop()
                    S(30)
                    time.sleep(1.2)
                    Stop()
                    W(30)
                    time.sleep(0.3)
                    Stop()
                    if distanceSensor_data[2]  > 100:
                        D(30)
                        time.sleep(0.6)
                    elif distanceSensor_data[3]  > 100:
                        A(30)
                        time.sleep(0.6)
                    elif distanceSensor_data[2]  < distanceSensor_data[3] :
                        A(30)
                        time.sleep(0.6)
                    elif distanceSensor_data[3]  < distanceSensor_data[2] :
                        D(30)
                        time.sleep(0.6)
                    else:
                        D(30)
                        time.sleep(0.6)
                    Stop()
                    W(30)
                    time.sleep(2.5)
                    Stop()
                    lastcom == ""
                else:
                    Stop()
                    S(30)
                    time.sleep(0.4)
                    Stop()
                    if distanceSensor_data[2]  > 100:
                        D(30)
                        time.sleep(0.6)
                    elif distanceSensor_data[3]  > 100:
                        A(30)
                        time.sleep(0.6)
                    elif distanceSensor_data[2]  < distanceSensor_data[3] :
                        A(30)
                        time.sleep(0.6)
                    elif distanceSensor_data[3]  < distanceSensor_data[2] :
                        D(30)
                        time.sleep(0.6)
                    else:
                        D(30)
                        time.sleep(0.8)
                    Stop()
                    W(30)
                    time.sleep(2.5)
                    Stop() 
                    lastcom == ""
            if time.time()-time_person > 30:
                W(30)
                yaw = 0
                time_person = time.time()
            elif time.time()-time_person > 5:
                if lastcom == "D":
                    D(30)
                elif lastcom == "A":
                    A(30)
                elif yaw <= 0:
                    D(30)
                    lastcom = "D"
                elif yaw > 0:
                    A(30)
                    lastcom = "A"
                time.sleep(0.5)
                Stop()
                time.sleep(1)     
            # print('ddd')      
        except:
            print("Error")
def openbin(com):
    if com == "off":
        for i in range(90,0,-2):
            kit.servo[4].angle = i
            time.sleep(0.005)
    elif com == "on":
        for i in range(0,90,2):
            kit.servo[4].angle = i
            time.sleep(0.005)

def get_sorted_dates(dates):
    sorted_dates = sorted(dates, key=lambda date: datetime.strptime(date, '%d/%m/%Y'))
    return sorted_dates

                

# Send Frame Camera 

def read_camera():
    global frame, cap,frameweb
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Break Camera")
            break
        frameweb = frame.copy()
        cv.rectangle(frameweb, (620,340), (660,380), (0, 255, 0), 2)
        time.sleep(0.033) 

def amplify_audio(audio_stream, gain_dB=10):
    # Convert the raw audio to AudioSegment for processing
    audio_segment = AudioSegment.from_file(BytesIO(audio_stream.get_wav_data()))
    # Amplify the audio
    amplified_audio = audio_segment + gain_dB
    # Convert back to audio data for recognition
    return sr.AudioData(amplified_audio.raw_data, audio_stream.sample_rate, audio_stream.sample_width)

def mic():
    global keyword,coms
    recognizer = sr.Recognizer()
    recognizer.energy_threshold = 700
    recognizer.dynamic_energy_threshold = True
    with sr.Microphone() as source:
        while True:
            if coms["Mode"] != 0:
                time.sleep(1)
                continue
            try:
                recognizer.adjust_for_ambient_noise(source, duration=1)
                audio_stream = recognizer.listen(source)

                amplified_audio = amplify_audio(audio_stream, gain_dB=5)  # Adjust gain as needed

                print("Google")
                text = recognizer.recognize_google(amplified_audio, language="th-TH", show_all=False)
                print("คำที่ตรวจจับได้คือ: {}".format(text))
                if keyword in text:
                    Stop()
                    playsound("iamhearu")
                    coms["Mode"] = 2
            except sr.UnknownValueError:
                print("ไม่สามารถตรวจจับคำพูด")
            except sr.RequestError as e:
                print("เกิดข้อผิดพลาดในการเชื่อมต่อกับ Google API: {}".format(e))
def writelog():
    global ina219,battery,coms,gohome
    timestart = time.time()
    timesave = time.time()
    sum = 0
    sumbat = 0
    count = 0
    while True:
        if time.time()-timestart >=1:
            timestart = time.time()
            bus_voltage = ina219.bus_voltage  
            sumbat += int((bus_voltage - 10.5) / 0.015)
            sum += ina219.power  
            count +=1
        if time.time()-timesave >= 60:
            timesave = time.time()
            current_date = datetime.now().strftime("%Y-%m-%d")
            folder_path = '/home/user/BinRobot/data/*.csv'  # เลือกเฉพาะไฟล์ .csv
            file_names = glob.glob(folder_path)
            if f"/home/user/BinRobot/data/{current_date}.csv" in file_names:
                watt = sum//count
                battery = sumbat//count
                if battery < gohome:
                    Stop()
                    playsound('gobackst')
                    coms['Mode'] = 4
                if int(abs(100-(int(distanceSensor_data[0])*100/25))) > 80:
                    Stop()
                    playsound('binfull')
                    coms["Mode"] = 4
                with open(f'/home/user/BinRobot/data/{current_date}.csv', 'a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([datetime.now().strftime("%d-%m-%y %H:%M:%S"), int(battery), int(watt)])
                sum = 0
                count = 0
                sumbat = 0
            else:
                watt = sum//count
                battery = sumbat//count
                with open(f'/home/user/BinRobot/data/{current_date}.csv', 'w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([datetime.now().strftime("%d-%m-%y %H:%M:%S"), int(battery), int(watt)])
                sum = 0
                count = 0
                sumbat = 0
        time.sleep(0.3)
# WebCam in Website
def generate_frames():
    global frameweb
    while True:
        if frameweb is None:
            continue
        ret, buffer = cv.imencode('.jpg', frameweb)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/camera')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
def get_ssid_linux():
    try:
        result = subprocess.check_output(["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"], encoding='utf-8')
        for line in result.splitlines():
            if line.startswith("yes:"):
                return line.split(":")[1]
        return "No active Wi-Fi connection found."
    except subprocess.CalledProcessError as e:
        return f"Error: {e}"

def get_ip():
    result = subprocess.run(['ip', 'addr', 'show', 'wlan0'], stdout=subprocess.PIPE)
    output = result.stdout.decode('utf-8')
    
    for line in output.splitlines():
        if 'inet ' in line:
            ip_address = line.split()[1].split('/')[0]  
            return ip_address
    return None

def check_net():
    global ip_server
    try:
        socket.create_connection(("www.google.com", 80), timeout=5)
        return True
    except OSError:
        return False    

def remove_all_wifi():
    try:
        result = subprocess.run(['sudo','nmcli', '-t', '-f', 'NAME', 'connection', 'show'], capture_output=True, text=True)
        connections = result.stdout.strip().split('\n')
        for connection in connections:
            if connection:
                subprocess.run(['sudo','nmcli', 'connection', 'delete', connection], check=True)
                print(f"ลบการเชื่อมต่อ {connection} เรียบร้อยแล้ว")
                
    except subprocess.CalledProcessError as e:
        print(f"เกิดข้อผิดพลาดในการลบการเชื่อมต่อ: {e}")

def wifi_manager():
    global coms,internet_con,ip_server
    while True:
        if check_net():
            if ip_server == None:
                ip_str = f"{get_ip()}"
                while ip_str == "192.168.4.1":
                    ip_str = f"{get_ip()}"
                ip_server = ip_str
                playsound("start_mc")
                print(ip_str)
                time.sleep(1)
                for x in ip_str:
                    if x == ".":
                        playsound("dot")
                    else:
                        playsound(x)
                coms['Mode'] = 0
                
            subprocess.run(['sudo', 'systemctl', 'stop', 'hostapd'])
            internet_con = True
            
        else:
            if coms["Mode"] != 3:
                remove_all_wifi()
                coms["Mode"] = 3
                subprocess.run(['sudo','systemctl', 'start', 'hostapd'])
                subprocess.run(['sudo','iptables', '-t', 'nat', '-I', 'PREROUTING', '-d', '34.32.47.10', '-p', 'tcp', '--dport', '34', '-j', 'DNAT', '--to-destination', '192.168.4.1:2000'])
        time.sleep(10)


@app.route('/api/setwifi2', methods=['GET'])
def set_wifi2():
    global internet_con
    data = {
            "ssid" : request.args.get('ssid'),
            "password" : request.args.get('password'),
            }
    if internet_con:
        return jsonify({"status":True})
    print("CONNECT")
    command = ['sudo','nmcli', 'device', 'wifi', 'connect', data["ssid"], 'password', data["password"]]
    print("CONNECT last")
    try:
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        print("WiFi connected successfully:", result.stdout)

        return jsonify({"status": True})
    except subprocess.CalledProcessError as e:
        print("Failed to connect to WiFi:", e.stderr)
        return jsonify({"status": False})

@app.route('/api/getdata', methods=['GET'])
def get_data():
    global ina219,coms,battery,batterypersenwatt,datebatterylist
    data = {}
    page = request.args.get('page')
    print(page)
    Status = ['Auto','Manual',"Follow","Stay","Charge","Stuck"]
    if page == "Dashboard":
        data["Batterylevel"] = int(battery)
        if batterypersenwatt == None and len(datebatterylist) == 0:
            batteryloss = 0
        elif batterypersenwatt == None:
            batterypersenwatt = wattperperseninmin(datebatterylist[0])
            batload = wattperperseninmin(datebatterylist[-1])
            batteryloss = batload-batterypersenwatt
            if batteryloss < 0:
               batteryloss = 0
        else:
           batteryloss =  wattperperseninmin(datebatterylist[-1]) - batterypersenwatt
           if batteryloss < 0:
               batteryloss = 0
        data["Batterydestroy"] = round(batteryloss,2)
        data["Trashlevel"]= int(abs(100-(int(distanceSensor_data[0])*100/25)))
        data["Status"]= Status[coms["Mode"]]
        data["Wifi"]= get_ssid_linux()
        return jsonify(data)
    elif page == "Setting":
        with open("/home/user/BinRobot/data/Setting.json", mode='r', encoding='utf-8') as file:
            data = json.load(file)
        print(data)
        return jsonify(data)
    else:
        return jsonify({"message": "No valid page specified"})
    

@app.route('/api/getGraph', methods=['GET'])
def get_date():
    global datebatterylist
    date = request.args.get('dateSelect')
    data = {
        "time":[],
        "battery":[],
        "powerUsage":[]
    }
    print(f"input = {date}")
    if date == "":
        folder_path = '/home/user/BinRobot/data/'
        datetime = []
        hourly_data = {}
        file_names = glob.glob(os.path.join(folder_path, "*.csv")) # ดึงเฉพาะไฟล์ที่มีนามสกุล .csv
        last_file_path = os.path.basename(file_names[-1])
        with open(folder_path+last_file_path, mode='r', newline='') as file:
            reader = file.readlines()
            for x in reader:
                split_data = x.split(',')
                hour = split_data[0].split(" ")[1].split(":")[0]
                battery = (int(split_data[1]))
                powerUsage = (int(split_data[2]))
                if hour not in hourly_data:
                    hourly_data[hour] = {"sum_battery":0,"sum_powerUsage":0,"count":0}
                hourly_data[hour]['sum_battery'] += battery
                hourly_data[hour]['sum_powerUsage'] += powerUsage
                hourly_data[hour]['count'] += 1
                
        avg_data = {}
        
        for hour, values in hourly_data.items():
            avg_battery = values['sum_battery'] / values['count']
            avg_powerUsage = values['sum_powerUsage'] / values['count']
            avg_data[hour] = {'avg_battery': avg_battery, 'avg_powerUsage': avg_powerUsage}
        for hour, avg in avg_data.items():
            data["time"].append(f"{hour}:00")
            data["battery"].append(int(avg["avg_battery"]))
            data["powerUsage"].append(int(avg["avg_powerUsage"]))
        for f in file_names:
            year, month, day = os.path.basename(f).replace(".csv","").split("-")
            datetime.append(f"{day}/{month}/{year}")

        sorted_dates = get_sorted_dates(datetime)
        print(sorted_dates)
        datebatterylist = sorted_dates.copy()
        data["datetime"] = sorted_dates 

        print(data)
        return jsonify(data)
    else:
        day,month,year = date.split("/")
        file_name = f"{year}-{month}-{day}.csv"
        folder_path = '/home/user/BinRobot/data/'
        datetime = []
        hourly_data = {}
        with open(folder_path+file_name, mode='r', newline='') as file:
            reader = file.readlines()
            for x in reader:
                split_data = x.split(',')
                hour = split_data[0].split(" ")[1].split(":")[0]
                battery = (int(split_data[1]))
                powerUsage = (int(split_data[2]))
                if hour not in hourly_data:
                    hourly_data[hour] = {"sum_battery":0,"sum_powerUsage":0,"count":0}
                hourly_data[hour]['sum_battery'] += battery
                hourly_data[hour]['sum_powerUsage'] += powerUsage
                hourly_data[hour]['count'] += 1
                
        avg_data = {}
        
        for hour, values in hourly_data.items():
            avg_battery = values['sum_battery'] / values['count']
            avg_powerUsage = values['sum_powerUsage'] / values['count']
            avg_data[hour] = {'avg_battery': avg_battery, 'avg_powerUsage': avg_powerUsage}
        for hour, avg in avg_data.items():
            data["time"].append(f"{hour}:00")
            data["battery"].append(int(avg["avg_battery"]))
            data["powerUsage"].append(int(avg["avg_powerUsage"]))
        print(data)
        return jsonify(data)
    # return jsonify({"message": "No valid page specified"}), 404

@app.route('/api/update', methods=['POST'])
def get_update():
    global distanceFix,gohome,keyword
    data = request.json  
    distanceFix = int(data['stopdistance'])
    gohome = int(data['gohome'])
    keyword = data['keyword']
    with open('/home/user/BinRobot/data/Setting.json', 'w') as file:
        json.dump(data, file, indent=4)
    print("update info succesfully.")
    return jsonify({'message': 'updated successfully!'})
    

async def main_asyncio():
    await main()

def get_gyro():
    global yaw,prev_time
    while True:
        try:
            gyro_data = sensor.get_gyro_data()

            gyro_data['y'] -= gyro_bias['y']

            curr_time = time.time()
            dt = curr_time - prev_time
            prev_time = curr_time

            delta_yaw = gyro_data['y'] * dt
            yaw += delta_yaw

            # print(f"Yaw: {yaw:.2f} degrees")
            time.sleep(0.01)
        except:
            time.sleep(0.1)

if __name__ == "__main__":
    
    camera_thread = threading.Thread(target=read_camera)
    camera_thread.daemon = True
    wifi_manager_thread = threading.Thread(target=wifi_manager)
    wifi_manager_thread.daemon = True
    automode_thread = threading.Thread(target=automode)
    automode_thread.daemon = True
    follow_thread = threading.Thread(target=follow)
    follow_thread.daemon = True
    Battery_thread = threading.Thread(target=Battery)
    Battery_thread.daemon = True
    Motor_thread = threading.Thread(target=get_gyro)
    Motor_thread.daemon = True
    Log_thread = threading.Thread(target=writelog)
    Log_thread.daemon = True
    interruptIR_thread = threading.Thread(target=interruptIR)
    interruptIR_thread.daemon = True
    get_sensor_thread = threading.Thread(target=get_sensor)
    get_sensor_thread.daemon = True
    mic_thread = threading.Thread(target=mic)
    mic_thread.daemon = True
    Log_thread.start()
    mic_thread.start()
    wifi_manager_thread.start()
    get_sensor_thread.start()
    Battery_thread.start()
    camera_thread.start()
    interruptIR_thread.start()
    automode_thread.start()
    follow_thread.start()
    Motor_thread.start()
    
    asyncio_thread = threading.Thread(target=asyncio.run, args=(main_asyncio(),))
    asyncio_thread.start()
    
    app.run(host='0.0.0.0', port=5000, threaded=True)
