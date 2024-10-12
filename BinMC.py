import threading
import time
import cv2 as cv
import json
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

import random
sensor = mpu6050(0x68,bus=0)
i2c_bus = busio.I2C(1, 0)
pin_br = 27  
pin_fr = 22  
pin_bl = 17  
pin_fl = 4   
modestop = False
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.output(20,0)
GPIO.output(21,0)

GPIO.output(20,1)
time.sleep(0.1)       
VL53L0X(i2c_bus,address=41).set_address(0x2A)
time.sleep(0.1)       
GPIO.output(21,1)
VL53L0X(i2c_bus,address=41).set_address(0x2B)
GPIO.output(20,1)
GPIO.output(21,1)

vl53 = VL53L0X(i2c=i2c_bus,address=0x2A)
vl532 = VL53L0X(i2c=i2c_bus,address=0x2B)

vl53.measurement_timing_budget = 50000
vl532.measurement_timing_budget = 50000
vl53.signal_rate_limit = 0.1
vl532.signal_rate_limit = 0.05

kit = ServoKit(i2c=i2c_bus,channels=8)
ina219 = INA219(i2c_bus,0x41)
ina219.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
ina219.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
ina219.bus_voltage_range = BusVoltageRange.RANGE_16V
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()


# subprocess.run(['sudo', 'iptables', '-t', 'nat', '-A', 'POSTROUTING', '-o', 'eth0', '-j', 'MASQUERADE'], check=True)

# subprocess.run(['sudo', 'iptables', '-t', 'nat', '-I', 'PREROUTING', '-d', 'BinMa.cpe.com', '-p', 'tcp', '--dport', '80', '-j', 'DNAT', '--to-destination', '192.168.4.1:80'], check=True)

# subprocess.run(['sudo', 'sh', '-c', 'iptables-save > /etc/iptables.ipv4.nat'], check=True)

# sensor = DistanceSensor(echo=6, trigger=5,max_distance=8)
# sensor2 = DistanceSensor(echo=16, trigger=26,max_distance=8)
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
    "Mode":0,
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
                    pygame.mixer.init()

                    pygame.mixer.music.load("sound/testsound.mp3")

                    pygame.mixer.music.play()

                    while pygame.mixer.music.get_busy():
                        continue
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

# Servo


# Sensor ระยะทาง

def Sensor_VL53L0X():
    global i2c_bus
    GPIO.output(20,0)
    GPIO.output(21,0)

    GPIO.output(20,1)
    time.sleep(0.1)       
    VL53L0X(i2c_bus,address=41).set_address(0x2A)
    time.sleep(0.1)       
    GPIO.output(21,1)
    VL53L0X(i2c_bus,address=41).set_address(0x2B)
    GPIO.output(20,1)
    GPIO.output(21,1)

    vl53 = VL53L0X(i2c=i2c_bus,address=0x2A)
    vl532 = VL53L0X(i2c=i2c_bus,address=0x2B)

    vl53.measurement_timing_budget = 50000
    vl532.measurement_timing_budget = 50000
    vl53.signal_rate_limit = 0.1
    vl532.signal_rate_limit = 0.05

# หามุมเลี้ยวหลบ


# Motor

# def DriveMotor():
#     global coms,led,led2,led3,led4,modear,vl53_1,vl53_2
#     while True:
#         if coms["Mode"] == 1:
#             time.sleep(1)
#             continue
#         # print(f"SPDL = {coms['SPDL']} , SPDR = {coms['SPDR']}")
#         if vl53_2.range/10 < 120 or vl53_1.range/10 < 50:
#             led.value = 0
#             led2.value = 0
#             led3.value = 0
#             led4.value = 0
            
#         if coms["Goto"] == "W":
#             #print("GO")
#             led.value = coms["SPDL"]
#             led2.value = 0
#             led3.value = 0
#             led4.value = coms["SPDR"]
#         elif coms["Goto"] == "D":
#             led.value = coms["SPDL"]
#             led2.value = 0
#             led3.value = coms["SPDR"]
#             led4.value = 0
#         elif coms["Goto"] == "A":
#             led.value = 0
#             led2.value = coms["SPDL"]
#             led3.value = 0
#             led4.value = coms["SPDR"]
#         elif coms["Goto"] == "S":
#             led.value = 0
#             led2.value = 0
#             led3.value = 0
#             led4.value = 0
#         try:
#             time.sleep(coms["Point"])
#             if coms['Break']:
#                 coms["SPDR"] = coms["SPDR"] - 0.1
#                 coms["SPDL"] = coms["SPDL"] - 0.1
#                 if coms["SPDL"] < 0:
#                     coms["SPDL"] = 0
                    
#                 if coms["SPDR"] < 0:
#                     coms["SPDR"] = 0           
#         except:
#             continue
        
# Google

# def Mediepie():
#     global frame,net,frameweb,coms,camera_y,kit,camera_x,modear,yaw,modestop,findperson
#     timereset = True
#     lastcom = "D"
    
#     timepro = 0
#     while True:
#         if frame is None or coms["Mode"] == 1:
#             time.sleep(1)
#             continue
#         recognizer = sr.Recognizer()
#         recognizer.energy_threshold = 100
#         recognizer.dynamic_energy_threshold = True

#         with sr.Microphone() as source:
#             print("กรุณาพูดคำที่ต้องการตรวจจับ...")

#             while True and not findperson:
#                 try:
#                     recognizer.adjust_for_ambient_noise(source, duration=1)
#                     audio_stream = recognizer.listen(source)

#                     # Amplify the audio stream
#                     amplified_audio = amplify_audio(audio_stream, gain_dB=10)  # Adjust gain as needed

#                     print("Google")
#                     text = recognizer.recognize_google(amplified_audio, language="th-TH", show_all=False)
#                     print("คำที่ตรวจจับได้คือ: {}".format(text))
#                     if "ถังขยะ" in text:
#                         sound_list = ['test2.wav',"test.wav","test3.wav"]
#                         # โหลดและปรับระดับความดังของไฟล์เสียง MP3 ด้วย PyDub
#                         mp3_audio = AudioSegment.from_mp3(sound_list[random.randint(1,1)])
#                         mp3_audio = mp3_audio + 10  # เพิ่มความดัง 10 dB

#                         wav_io = BytesIO()
#                         mp3_audio.export(wav_io, format="wav")

#                         wav_io.seek(0)
#                         pygame.mixer.init()
#                         pygame.mixer.music.load(wav_io, 'wav')

#                         pygame.mixer.music.play()

#                         while pygame.mixer.music.get_busy():
#                             continue
#                         findperson = True
#                         break
#                 except sr.UnknownValueError:
#                     print("ไม่สามารถตรวจจับคำพูด")
#                 except sr.RequestError as e:
#                     print("เกิดข้อผิดพลาดในการเชื่อมต่อกับ Google API: {}".format(e))
#                 except KeyboardInterrupt:
#                     print("โปรแกรมถูกหยุด")
#                     break
#         frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#         t = time.time_ns()
#         results = holistic.process(frame_rgb)
#         if results.pose_landmarks:
#             # ตรวจจับหลายคน
#             landmarks = pose_landmarks.landmark

#             left_hand_y = landmarks[mp_pose.PoseLandmark.LEFT_WRIST].y
#             right_hand_y = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].y
#             head_y = landmarks[mp_pose.PoseLandmark.NOSE].y
#             head_x = landmarks[mp_pose.PoseLandmark.NOSE].x
#             left_wrist = landmarks[mp_pose.PoseLandmark.LEFT_WRIST]
#             right_wrist = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST]
#             # ตำแหน่งของศีรษะและข้อเท้า
#             head = landmarks[mp_pose.PoseLandmark.NOSE]
#             left_ankle = landmarks[mp_pose.PoseLandmark.LEFT_ANKLE]
#             right_ankle = landmarks[mp_pose.PoseLandmark.RIGHT_ANKLE]

#             # คำนวณความกว้างจากระยะระหว่างมือซ้ายและมือขวา
#             width = abs(right_wrist.x - left_wrist.x) * frame.shape[1]  # แปลงเป็นพิกเซล
#             # คำนวณความสูงจากระยะระหว่างศีรษะและข้อเท้า
#             height = abs((left_ankle.y + right_ankle.y) / 2 - head.y) * frame.shape[0]  # แปลงเป็นพิกเซล
#             # Check if either hand is above the head for this person
#             print(f"Person {i+1} Full Body Width: {width:.2f} pixels, Full Body Height: {height:.2f} pixels")
#                 if left_hand_y < head_y or right_hand_y < head_y:
                    
#                     if timereset:
#                         sound_list = ['test2.wav', "test.wav", "test3.wav"]
#                         # โหลดและปรับระดับความดังของไฟล์เสียง MP3 ด้วย PyDub
#                         mp3_audio = AudioSegment.from_mp3(sound_list[random.randint(2, 2)])
#                         mp3_audio = mp3_audio + 10  # เพิ่มความดัง 10 dB

#                         wav_io = BytesIO()
#                         mp3_audio.export(wav_io, format="wav")

#                         wav_io.seek(0)
#                         pygame.mixer.init()
#                         pygame.mixer.music.load(wav_io, 'wav')

#                         pygame.mixer.music.play()

#                         # รอให้เสียงเล่นเสร็จ
#                         while pygame.mixer.music.get_busy():
#                             continue
#                         timereset = False
#                     else:
#                         findperson = True
#                         modestop = True
#                         timepro = time.time()
#                         lastcom = ""
#                         if head_y < 200:
#                             camera_y += 2
#                             kit.servo[0].angle = int(camera_y)
#                         print(100-(((1280 * 720 - width*height)/(1280 * 720))*100))
#                         if  100-(((1280 * 720 - width*height)/(1280 * 720))*100) > 50:
#                             yaw = 0
#                             distance1 = 120 if vl53.range/10 >= 120 else vl53.range/10
#                             distance2 = 120 if vl532.range/10 >= 120 else vl532.range/10
#                             if distance1 < 100:
#                                 Stop()
#                             distance1_sum = distance1 
#                             distance2_sum = distance2
#                             time.sleep(0.1)
#                             for i in range(1):
#                                 distance1temp = 120 if vl53.range/10 >= 120 else vl53.range/10
#                                 distance2temp = 120 if vl532.range/10 >= 120 else vl532.range/10
                                
#                                 distance1_sum += distance1temp
#                                 distance2_sum += distance2temp
#                                 time.sleep(0.1)
#                             distance1 = distance1_sum / 2
#                             distance2 = distance2_sum / 2

#                             print("Averaged Distance1: ", distance1)
#                             print("Averaged Distance2: ", distance2)
#                             while distance1 > 30:
#                                 if yaw < -10:
#                                     D(20)
#                                 elif yaw > 10:
#                                     A(20)
#                                 else:
#                                     W(20)
#                                 time.sleep(0.1)
#                                 distance1 = 120 if vl53.range/10 >= 120 else vl53.range/10
#                             Stop()  
#                             print("person off")
#                             camera_y = 100
#                             kit.servo[0].angle = int(camera_y)
#                             modestop = True
#                             openbin("on")
#                             time.sleep(10)
#                             openbin("off")
#                             modestop = False
#                             findperson = False
#                             timereset = True
#                             Stop()
#                             break
#                         if ((head_x) > 400 and (head_x) < 880) and coms["Mode"] == 0:
#                             print("person W")
#                             yaw = 0
#                             W(30)
#                         elif (head_x <= 400) and coms["Mode"] == 0:
#                             print("person A")
#                             A(30)
#                             time.sleep(0.2)
#                             Stop()
#                         elif (head_x > 880) and coms["Mode"] == 0:
#                             print("person D")
#                             D(30)
#                             time.sleep(0.2)
#                             Stop()
#                         modestop = False
#                         break
#         if time.time() - timepro > 10 and findperson:
#             modestop = True
#             modear = True
#             if lastcom == "D":
#                 D(30)
#             elif lastcom == "A":
#                 A(30)
#             elif yaw <= 0:
#                 D(30)
#                 lastcom = "D"
#             elif yaw > 0:
#                 A(30)
#                 lastcom = "A"
#             time.sleep(0.5)
#             Stop()
#             time.sleep(1)

                    

def amplify_audio(audio_stream, gain_dB=10):
    # Convert the raw audio to AudioSegment for processing
    audio_segment = AudioSegment.from_file(BytesIO(audio_stream.get_wav_data()))
    # Amplify the audio
    amplified_audio = audio_segment + gain_dB
    # Convert back to audio data for recognition
    return sr.AudioData(amplified_audio.raw_data, audio_stream.sample_rate, audio_stream.sample_width)


findperson = True
def Yolo_tiny():
    global frame,net,frameweb,coms,camera_y,kit,camera_x,modear,yaw,modestop,findperson,timereset
    
    lastcom = ""
    
    timepro = time.time()
    while True:
        if frame is None or coms["Mode"] == 1:
            time.sleep(1)
            continue
        

        # with sr.Microphone() as source:
        #     recognizer = sr.Recognizer()
        #     recognizer.energy_threshold = 100
        #     recognizer.dynamic_energy_threshold = False
        #     while True and timereset:
        #         try:
                    
        #             recognizer.adjust_for_ambient_noise(source, duration=1)
        #             audio_stream = recognizer.listen(source)

        #             # Amplify the audio stream
        #             amplified_audio = amplify_audio(audio_stream, gain_dB=40)  # Adjust gain as needed

        #             print("Google")
        #             text = recognizer.recognize_google(amplified_audio, language="th-TH", show_all=False)
        #             print("คำที่ตรวจจับได้คือ: {}".format(text))
        #             if "ถังขยะ" in text:
        #                 sound_list = ['test2.wav',"test.wav","test3.wav"]
        #                 # โหลดและปรับระดับความดังของไฟล์เสียง MP3 ด้วย PyDub
        #                 mp3_audio = AudioSegment.from_mp3(sound_list[random.randint(2,2)])
        #                 mp3_audio = mp3_audio + 0  # เพิ่มความดัง 10 dB

        #                 wav_io = BytesIO()
        #                 mp3_audio.export(wav_io, format="wav")

        #                 wav_io.seek(0)
        #                 pygame.mixer.init()
        #                 pygame.mixer.music.load(wav_io, 'wav')

        #                 pygame.mixer.music.play()

        #                 while pygame.mixer.music.get_busy():
        #                     continue
        #                 findperson = True
        #                 timereset = False
        #                 break
        #         except sr.UnknownValueError:
        #             print("ไม่สามารถตรวจจับคำพูด")
        #         except sr.RequestError as e:
        #             print("เกิดข้อผิดพลาดในการเชื่อมต่อกับ Google API: {}".format(e))
        #         except KeyboardInterrupt:
        #             print("โปรแกรมถูกหยุด")
        #             break
        try:
            height, width = frame.shape[:2]
            blob = cv.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
            net.setInput(blob)
            layer_names = net.getLayerNames()
            output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
            detections = net.forward(output_layers)

            boxes, confidences, class_ids = [], [], []
            for out in detections:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.5:
                        box = detection[0:4] * np.array([width, height, width, height])
                        (centerX, centerY, w, h) = box.astype("int")
                        x = int(centerX - (w / 2))
                        y = int(centerY - (h / 2))
                        boxes.append([x, y, int(w), int(h)])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)

            indices = cv.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
            if len(indices) > 0:
                for i in indices.flatten():
                    x, y, w, h = boxes[i]
                    label = str(classes[class_ids[i]])
                    confidence = confidences[i]
                    color = (0, 255, 0)
                    cv.rectangle(frameweb, (x, y), (x + w, y + h), color, 2)
                    if label == "person" and modear:
                        # roi = frame[y:y+h, x:x+w]  # ตัดส่วนที่ตรวจจับได้

                        # # ใช้ MediaPipe Pose กับภาพที่ถูกตัดออกมา
                        # roi_rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
                        # results = pose.process(roi_rgb)
                        # if results.pose_landmarks:
                        #     landmarks = results.pose_landmarks.landmark

                        #     left_hand_y = landmarks[mp_pose.PoseLandmark.LEFT_WRIST].y
                        #     right_hand_y = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].y
                        #     left_shoulder_y = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y
                        #     right_shoulder_y = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y
                        #     head = landmarks[mp_pose.PoseLandmark.NOSE].y
                        # if left_hand_y < head or right_hand_y < head:
                        findperson = True
                        modestop = False
                        timepro = time.time()
                        lastcom = ""
                        print(f"x = {x} w = {w} x+w = {x+w}")
                        print(f"y = {y} h = {h} y+h = {y+h}")
                        if y < 300:
                            camera_y += 3
                            kit.servo[0].angle = int(camera_y)
                        print(100-(((1280 * 720 - w*h)/(1280 * 720))*100))
                        if  100-(((1280 * 720 - w*h)/(1280 * 720))*100) > 40:
                            modestop = True
                            if ((x+(w//2)) > 400 and (x+(w//2)) < 880) and coms["Mode"] == 0:
                                yaw = 0
                            elif ((x+(w//2)) <= 400) and coms["Mode"] == 0:
                                yaw = 0+((640-(x+(w//2)))/12)
                            elif ((x+(w//2)) > 880) and coms["Mode"] == 0:
                                yaw = 0-(((x+(w//2))-640)/12)
                            else:
                                yaw = 0
                            distance1 = 120 if vl53.range/10 >= 120 else vl53.range/10
                            distance2 = 120 if vl532.range/10 >= 120 else vl532.range/10
                            if distance1 < 100:
                                Stop()
                            distance1_sum = distance1 
                            distance2_sum = distance2
                            time.sleep(0.05)
                            for i in range(1):
                                distance1temp = 120 if vl53.range/10 >= 120 else vl53.range/10
                                distance2temp = 120 if vl532.range/10 >= 120 else vl532.range/10
                                
                                distance1_sum += distance1temp
                                distance2_sum += distance2temp
                                time.sleep(0.1)
                            distance1 = distance1_sum / 2
                            distance2 = distance2_sum / 2

                            print("Averaged Distance1: ", distance1)
                            print("Averaged Distance2: ", distance2)
                            modestop = True
                            while distance1 > 30:
                                if yaw < -10:
                                    D(20)
                                elif yaw > 10:
                                    A(20)
                                else:
                                    W(20)
                                time.sleep(0.1)
                                distance1 = 120 if vl53.range/10 >= 120 else vl53.range/10
                            Stop()  
                            print("person off")
                            camera_y = 100
                            kit.servo[0].angle = int(camera_y)
                            openbin("on")
                            time.sleep(30)
                            openbin("off")
                            modestop = False
                            findperson = False
                            Stop()
                            break
                        if ((x+(w//2)) > 400 and (x+(w//2)) < 880) and coms["Mode"] == 0:
                            print("person W")
                            yaw = 0
                            W(30)
                        elif ((x+(w//2)) <= 400) and coms["Mode"] == 0:
                            print("person A")
                            A(30)
                            time.sleep(0.15)
                            Stop()
                        elif ((x+(w//2)) > 880) and coms["Mode"] == 0:
                            print("person D")
                            D(30)
                            time.sleep(0.15)
                            Stop()
                        modestop = False
                        break
        
        # print(coms["Break"])
        # print(f"Time Prosess {(time.time_ns()-timepro)/1000000}")
        except:
            print("Error")
        if time.time() - timepro > 5 and findperson:
            modestop = True
            modear = True
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

def sensorDe():
    global yaw,modear,modestop,findperson,timereset
    while True:
        try:
            if modestop:
                time.sleep(0.5)
                continue
            distance1 = 120 if vl53.range/10 >= 120 else vl53.range/10
            distance2 = 120 if vl532.range/10 >= 120 else vl532.range/10

            distance1_sum = distance1 
            distance2_sum = distance2
            time.sleep(0.05)
            for i in range(2):
                distance1temp = 120 if vl53.range/10 >= 120 else vl53.range/10
                distance2temp = 120 if vl532.range/10 >= 120 else vl532.range/10
                
                distance1_sum += distance1temp
                distance2_sum += distance2temp
                time.sleep(0.05)
            distance1 = distance1_sum / 3
            distance2 = distance2_sum / 3

            print("Averaged Distance1: ", distance1)
            print("Averaged Distance2: ", distance2)
            if distance1 < 40 or distance2 < 90:
                modear = False
                Stop()
                S(30)
                time.sleep(1)
                if yaw <= 0 and findperson:
                    D(30)
                    time.sleep(1)
                elif yaw > 0 and findperson:
                    A(30)
                    time.sleep(1)
                else:
                    A(40)
                    time.sleep(0.8)
                Stop()
                W(30)
                time.sleep(2.5)
                Stop()
            elif modestop:
                modear = True
                W(30)
        except:
            Stop()
            Sensor_VL53L0X()
def openbin(com):
    if com == "off":
        for i in range(90,0,-2):
            kit.servo[4].angle = i
            time.sleep(0.005)
    elif com == "on":
        for i in range(0,100,2):
            kit.servo[4].angle = i
            time.sleep(0.005)
def Yolo():
    mode = False
    tem = 0
    global model,frame,coms,camera_y,kit
    while True:
        if frame is None or coms["Mode"] == 1:
            time.sleep(1)
            continue
        results = model.predict(frame,conf=0.6,classes=(0))
        print(f"c y = {camera_y}")
        for x in results[0].boxes:
            if results[0].names[int(x.cls)] != "person":
                continue
            xy = x.xyxy.tolist()
            h,w,c = frame.shape
            print(h)
            print(xy[0][3])
            if mode:
                mode = False
                tem = abs(tem-xy[0][3])
                der = 2/tem
                he = (xy[0][3]-(h/2))*der

                angle_radians = math.radians(90-(abs(camera_y)-abs(he)))
                tan_value = math.tan(angle_radians)
                height = 80
                a = height / tan_value
                print(f"tem = {tem} der = {der} he = {he} angle_radians = {angle_radians} tan_value = {tan_value} abs(camera_y)+abs(he) = {abs(camera_y)-abs(he)}")
                print(camera_y)
                camera_y = 100
                kit.servo[0].angle = camera_y
                print(f"c = {a}")
                time.sleep(5)
                tem = 0
                continue
            if xy[0][3] > h - (h//15):
                camera_y = camera_y - 2
                if camera_y <= 0:
                    camera_y = 90
                kit.servo[0].angle = camera_y
                # time.sleep(0.5)
            elif not mode and tem == 0:
                tem = xy[0][3]
                camera_y = camera_y - 2
                if camera_y <= 0:
                    camera_y = 90
                kit.servo[0].angle = camera_y
                mode = True
                time.sleep(1)
                
            if (xy[0][0] < 620 or xy[0][2] > 660) and coms["Mode"] == 0:
                coms["Goto"] = "W"
                coms["Point"] = 0
            elif (xy[0][0] > 620) and coms["Mode"] == 0:
                coms["Goto"] = "D"
                coms["Point"] = xy[0][0] - 620
            elif (xy[0][2] < 660) and coms["Mode"] == 0:
                coms["Goto"] = "A"
                coms["Point"] = 660 - xy[0][2]
            else:
                coms["Goto"] = "S"
                coms["Point"] = 0
                

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
    
@app.route('/getdatadb', methods=['GET'])
def senddataDashboard():
    global ina219
    data = {
        "Battery" : 0,
        "Trash": 0,
        "Batloss": 0,
        "SSID":"",
        "Status":"",
    }
    bus_voltage = ina219.bus_voltage  
    shunt_voltage = ina219.shunt_voltage 
    current = ina219.current  
    power = ina219.power  
    battery = (bus_voltage - 11.1) / 0.015
    print(bus_voltage)
    print(current)
    data["Battery"] = battery
    data["Trash"] = random.randint(0,100)
    data["SSID"] = get_ssid_linux()
    data["Status"] = "Stay"
    data["Batloss"] = 0
    return jsonify(data)



# Start Programs

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
    Yolo_thread = threading.Thread(target=Yolo_tiny)
    Yolo_thread.daemon = True
    Motor_thread = threading.Thread(target=get_gyro)
    Motor_thread.daemon = True
    Sensor_thread = threading.Thread(target=sensorDe)
    Sensor_thread.daemon = True
    camera_thread.start()
    Yolo_thread.start()
    Motor_thread.start()
    Sensor_thread.start()
    
    asyncio_thread = threading.Thread(target=asyncio.run, args=(main_asyncio(),))
    asyncio_thread.start()
    
    app.run(host='0.0.0.0', port=8000, threaded=True)
