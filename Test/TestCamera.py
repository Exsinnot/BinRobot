import threading
import time
import cv2 as cv
import os
from flask import Flask, request, jsonify ,Response
from flask_socketio import SocketIO, emit
import asyncio
from gpiozero import LED,PWMLED
from ultralytics import YOLO
import json
import websockets
import numpy as np
import subprocess
import math
from adafruit_servokit import ServoKit
import random
kit = ServoKit(channels=8)
app = Flask(__name__)
camera_x = 90
camera_y = 95
kit.servo[0].angle = camera_x #X
kit.servo[4].angle = camera_y #Y
frame = None
cap = None
frameweb = None
def read_camera():
    global frame, cap,frameweb,camera_y
    turn = True
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frameweb = frame.copy()
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        edges = cv.Canny(gray, 50, 150, apertureSize=3)

        lines = cv.HoughLinesP(edges, 1, np.pi/180, threshold=100, minLineLength=100, maxLineGap=8)

        if lines is not None:
            print(lines)
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv.line(frameweb, (x1, y1), (x2, y2), (0, 255, 0), 2)
        YC = (camera_y * 4)
        x2L = (YC - 720) / (-1.3) + 100
        x2R = (YC - 720) / (1.3) + 1180
        cv.line(frameweb, (100, 720), (int(x2L), YC), (0, 255, 0), 2)
        cv.line(frameweb, (1180, 720), (int(x2R), YC), (0, 255, 0), 2)
        
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



if __name__ == "__main__":
    camera_thread = threading.Thread(target=read_camera)
    camera_thread.start()
    
    app.run(host='0.0.0.0', port=8000, threaded=True)
