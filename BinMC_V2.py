import threading
import time
import cv2 as cv
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
import threading
import cv2
import mediapipe as mp
import time
from pydub import AudioSegment
from io import BytesIO
import pygame
import random
import cv2
import mediapipe as mp
import time
from pydub import AudioSegment
from io import BytesIO
import pygame
from datetime import datetime
import random
sensor = mpu6050(0x68,bus=0)
i2c_bus = busio.I2C(1, 0)
pin_br = 27  
pin_fr = 22  
pin_bl = 17  
pin_fl = 4   

GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
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
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
sensor1 = DistanceSensor(echo=11, trigger=9,max_distance=8) #IN
sensor2 = DistanceSensor(echo=24, trigger=23,max_distance=8) #F
sensor3 = DistanceSensor(echo=14, trigger=15,max_distance=8) #R
sensor4 = DistanceSensor(echo=6, trigger=13,max_distance=8) #F

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_br, GPIO.OUT)
GPIO.setup(pin_fr, GPIO.OUT)
GPIO.setup(pin_bl, GPIO.OUT)
GPIO.setup(pin_fl, GPIO.OUT)


pwm_br = GPIO.PWM(pin_br, 100)  
pwm_fr = GPIO.PWM(pin_fr, 100)
pwm_bl = GPIO.PWM(pin_bl, 100)
pwm_fl = GPIO.PWM(pin_fl, 100)

pwm_br.start(0)
pwm_fr.start(0)
pwm_bl.start(0)
pwm_fl.start(0)

ip = ""
ports = 8001
CONNECTION = set()
camera_x = 90
camera_y = 120
kit.servo[0].angle = 120 #X
kit.servo[4].angle = 0 #Y
modear = True
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
    "Mode":4,
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


# Sensor ระยะทาง

def get_bounding_box(hand_landmarks, frame_width, frame_height):
    min_x = min([landmark.x for landmark in hand_landmarks.landmark]) * frame_width
    min_y = min([landmark.y for landmark in hand_landmarks.landmark]) * frame_height
    max_x = max([landmark.x for landmark in hand_landmarks.landmark]) * frame_width
    max_y = max([landmark.y for landmark in hand_landmarks.landmark]) * frame_height
    return (min_x, min_y), (max_x, max_y)
def automode():
    global frame, yaw, kit, coms, mp_hands
    mp_hands = mp.solutions.hands
    with mp_hands.Hands(
            static_image_mode=True,
            max_num_hands=2,
            min_detection_confidence=0.5
        ) as hands:
        while True:
            if frame is None or coms["Mode"] != 0:
                Stop()
                # print("Error: No frame available for detection.")
                time.sleep(1)
                continue

            frame = cv2.flip(frame, 1) 
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
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
            if sensor2.distance * 100  < 65 or sensor3.distance * 100 < 20 or sensor4.distance * 100 < 20:
                if sensor2.distance * 100  < 65:
                    Stop()
                    S(30)
                    time.sleep(1.2)
                    Stop()
                    W(30)
                    time.sleep(0.3)
                    Stop()
                    if sensor3.distance * 100  > 100:
                        D(30)
                        time.sleep(0.8)
                    elif sensor4.distance * 100  > 100:
                        A(30)
                        time.sleep(0.8)
                    elif sensor3.distance * 100  < sensor4.distance * 100 :
                        A(30)
                        time.sleep(0.8)
                    elif sensor3.distance * 100  < sensor4.distance * 100 :
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
                    if sensor3.distance * 100  > 100:
                        D(30)
                        time.sleep(0.8)
                    elif sensor4.distance * 100  > 100:
                        A(30)
                        time.sleep(0.8)
                    elif sensor3.distance * 100  < sensor4.distance * 100 :
                        A(30)
                        time.sleep(0.8)
                    elif sensor3.distance * 100  < sensor4.distance * 100 :
                        D(30)
                        time.sleep(0.8)
                    else:
                        D(30)
                        time.sleep(1)
                    Stop()
            else:
                W(35)

            

def openbin(com):
    if com == "off":
        for i in range(90,0,-2):
            kit.servo[4].angle = i
            time.sleep(0.005)
    elif com == "on":
        for i in range(0,90,2):
            kit.servo[4].angle = i
            time.sleep(0.005)

                

# Send Frame Camera 

def read_camera():
    global frame, cap,frameweb
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frameweb = frame.copy()
        cv.rectangle(frameweb, (620,340), (660,380), (0, 255, 0), 2)
        time.sleep(0.033) 

def writelog():
    global ina219,battery
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
    
@app.route('/api/getdata', methods=['GET'])
def get_data():
    global ina219,coms,battery
    data = {}
    page = request.args.get('page')
    print(page)
    Status = ['Auto','Manual',"Follow","Stay","Change"]
    if page == "Dashboard":
        data["Batterylevel"] = int(battery)
        data["Batterydestroy"] = 0 #ยังไม่ได้เขียน
        data["Trashlevel"]= int(abs(100-(int(sensor1.distance*100)*100/25)))
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
                hour = split_data[0][:2]
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
        data["datetime"] = datetime 
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
                hour = split_data[0][:2]
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
        return jsonify(data)
    # return jsonify({"message": "No valid page specified"}), 404

@app.route('/api/update', methods=['POST'])
def get_update():
    data = request.json  
    with open("Setting.csv", mode='w',encoding='utf-8') as file:
        file.write(data["name"]+","+str(data["gohome"])+","+str(data["stopdistance"]))
    print("update info succesfully.")
    return jsonify({'message': 'CSV updated successfully!'})
    

async def main_asyncio():
    await main()

def get_gyro():
    global yaw,prev_time
    while True:
        gyro_data = sensor.get_gyro_data()

        gyro_data['y'] -= gyro_bias['y']

        curr_time = time.time()
        dt = curr_time - prev_time
        prev_time = curr_time

        delta_yaw = gyro_data['y'] * dt
        yaw += delta_yaw

        # print(f"Yaw: {yaw:.2f} degrees")
        time.sleep(0.01)

if __name__ == "__main__":
    camera_thread = threading.Thread(target=read_camera)
    camera_thread.daemon = True
    automode_thread = threading.Thread(target=automode)
    automode_thread.daemon = True
    Motor_thread = threading.Thread(target=get_gyro)
    Motor_thread.daemon = True
    Log_thread = threading.Thread(target=writelog)
    Log_thread.daemon = True
    Log_thread.start()
    camera_thread.start()
    automode_thread.start()
    Motor_thread.start()
    
    asyncio_thread = threading.Thread(target=asyncio.run, args=(main_asyncio(),))
    asyncio_thread.start()
    
    app.run(host='0.0.0.0', port=5000, threaded=True)
