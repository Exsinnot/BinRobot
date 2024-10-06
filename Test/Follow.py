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
# เรียกใช้โมดูล MediaPipe Pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

# เปิดกล้อง
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
mode = True
sensor = mpu6050(0x68,bus=0)
i2c = busio.I2C(1, 0)
pin_br = 27  
pin_fr = 22  
pin_bl = 17  
pin_fl = 4   

GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.output(20,0)
GPIO.output(21,0)

GPIO.output(20,1)
time.sleep(0.1)       
VL53L0X(i2c,address=41).set_address(0x2A)
time.sleep(0.1)       
GPIO.output(21,1)
VL53L0X(i2c,address=41).set_address(0x2B)
GPIO.output(20,1)
GPIO.output(21,1)

vl53 = VL53L0X(i2c=i2c,address=0x2A)
vl532 = VL53L0X(i2c=i2c,address=0x2B)

vl53.measurement_timing_budget = 50000
vl532.measurement_timing_budget = 50000
vl53.signal_rate_limit = 0.01
vl532.signal_rate_limit = 0.01


GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_br, GPIO.OUT)
GPIO.setup(pin_fr, GPIO.OUT)
GPIO.setup(pin_bl, GPIO.OUT)
GPIO.setup(pin_fl, GPIO.OUT)


pwm_br = GPIO.PWM(pin_br, 1000)  
pwm_fr = GPIO.PWM(pin_fr, 1000)
pwm_bl = GPIO.PWM(pin_bl, 1000)
pwm_fl = GPIO.PWM(pin_fl, 1000)

pwm_br.start(0)
pwm_fr.start(0)
pwm_bl.start(0)
pwm_fl.start(0)

yaw = 0
prev_time = time.time()
frame = None
cap = None
frameweb = None
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



def W(Speed):
    pwm_br.ChangeDutyCycle(0)              
    pwm_fr.ChangeDutyCycle(Speed)      
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
 
def SetupVl53(): 
    GPIO.output(20,0)
    GPIO.output(21,0)

    GPIO.output(20,1)
    time.sleep(0.1)       
    VL53L0X(i2c,address=41).set_address(0x2A)
    time.sleep(0.1)       
    GPIO.output(21,1)
    VL53L0X(i2c,address=41).set_address(0x2B)
    GPIO.output(20,1)
    GPIO.output(21,1)

    vl53 = VL53L0X(i2c=i2c,address=0x2A)
    vl532 = VL53L0X(i2c=i2c,address=0x2B)

    vl53.measurement_timing_budget = 50000
    vl532.measurement_timing_budget = 50000
    vl53.signal_rate_limit = 0.01
    vl532.signal_rate_limit = 0.01


def main():
    global yaw,mode,frame
    while True:
        if frame is None:
            continue
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        t = time.time_ns()
        distance1 = 120 if vl53.range/10 >= 120 else vl53.range/10
        distance2 = 120 if vl532.range/10 >= 120 else vl532.range/10
        results = pose.process(frame_rgb)
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark

            left_hand_y = landmarks[mp_pose.PoseLandmark.LEFT_WRIST].y
            right_hand_y = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].y
            left_shoulder_y = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y
            right_shoulder_y = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y
            head = landmarks[mp_pose.PoseLandmark.NOSE].y
            headx = landmarks[mp_pose.PoseLandmark.NOSE].x
            print("person")
            if left_hand_y < head or right_hand_y < head:
                if mode:
                    sound_list = ['test2.wav',"test.wav","test3.wav"]
                    # โหลดและปรับระดับความดังของไฟล์เสียง MP3 ด้วย PyDub
                    mp3_audio = AudioSegment.from_mp3(sound_list[random.randint(2,2)])
                    mp3_audio = mp3_audio + 10  # เพิ่มความดัง 10 dB

                    wav_io = BytesIO()
                    mp3_audio.export(wav_io, format="wav")

                    wav_io.seek(0)
                    pygame.mixer.init()
                    pygame.mixer.music.load(wav_io, 'wav')

                    pygame.mixer.music.play()

                    # รอให้เสียงเล่นเสร็จ
                    while pygame.mixer.music.get_busy():
                        continue
                    mode = False
                else:
                    print("Hand Up")
                    if distance1 < 50 or distance2 < 100:
                        Stop()
                        distance1 = 120 if vl53.range/10 >= 120 else vl53.range/10
                        distance2 = 120 if vl532.range/10 >= 120 else vl532.range/10

                        distance1_sum = distance1 
                        distance2_sum = distance2

                        for i in range(3):
                            distance1temp = 120 if vl53.range/10 >= 120 else vl53.range/10
                            distance2temp = 120 if vl532.range/10 >= 120 else vl532.range/10
                            
                            distance1_sum += distance1temp
                            distance2_sum += distance2temp
                            time.sleep(0.05)
                        distance1 = distance1_sum / 4
                        distance2 = distance2_sum / 4

                        print("Averaged Distance1: ", distance1)
                        print("Averaged Distance2: ", distance2)
                        if distance1 < 50 or distance2 < 100:
                            S(30)
                            time.sleep(1)
                            if yaw <= 0:
                                A(30)
                                time.sleep(0.5)
                            else:
                                D(30)
                                time.sleep(0.5)
                            Stop()
                    elif headx > 600 and headx < 680:
                        yaw = 0
                        print("W")
                        W(30)
                    elif headx < 600:
                        A(30)
                        print("A")
                        time.sleep(0.1)
                        Stop()
                    elif headx > 680:
                        D(30)
                        print("D")
                        time.sleep(0.1)
                        Stop()
            
            # mp.solutions.drawing_utils.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        # print((time.time_ns() - t)/(10**6))
        if distance1 < 50 or distance2 < 100:
            Stop()
            distance1 = 120 if vl53.range/10 >= 120 else vl53.range/10
            distance2 = 120 if vl532.range/10 >= 120 else vl532.range/10

            distance1_sum = distance1 
            distance2_sum = distance2

            for i in range(3):
                distance1temp = 120 if vl53.range/10 >= 120 else vl53.range/10
                distance2temp = 120 if vl532.range/10 >= 120 else vl532.range/10
                
                distance1_sum += distance1temp
                distance2_sum += distance2temp
                time.sleep(0.05)
            distance1 = distance1_sum / 4
            distance2 = distance2_sum / 4

            print("Averaged Distance1: ", distance1)
            print("Averaged Distance2: ", distance2)
            if distance1 < 50 or distance2 < 100:
                S(30)
                time.sleep(1)
                if yaw <= 0:
                    A(30)
                    time.sleep(0.5)
                else:
                    D(30)
                    time.sleep(0.5)
                Stop()
        else:
            W(30)

def read_camera():
    global frame, cap,frameweb
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frameweb = frame.copy()
        cv2.rectangle(frameweb, (620,340), (660,380), (0, 255, 0), 2)
        time.sleep(0.033) 



if __name__ == "__main__":
    camera_thread = threading.Thread(target=read_camera)
    camera_thread.daemon = True
    main_thread = threading.Thread(target=main)
    gyro_thread = threading.Thread(target=get_gyro)
    main_thread.start()
    gyro_thread.start()
    camera_thread.start()
    main_thread.join()
    gyro_thread.join()
    camera_thread.join()
