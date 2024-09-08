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
# subprocess.run(['sudo', 'iptables', '-t', 'nat', '-A', 'POSTROUTING', '-o', 'eth0', '-j', 'MASQUERADE'], check=True)

# subprocess.run(['sudo', 'iptables', '-t', 'nat', '-I', 'PREROUTING', '-d', 'BinMa.cpe.com', '-p', 'tcp', '--dport', '80', '-j', 'DNAT', '--to-destination', '192.168.4.1:80'], check=True)

# subprocess.run(['sudo', 'sh', '-c', 'iptables-save > /etc/iptables.ipv4.nat'], check=True)

ip = ""
ports = 8001
CONNECTION = set()
led = PWMLED(27) #เดินหน้า ซ้าย
led2 = PWMLED(22) #ถอยหลัง ซ้าย
led3 = PWMLED(4) #ถอยหลัง ขวา
led4 = PWMLED(17) #เดินหน้า ขวา
camera_x = 90
camera_y = 90
kit.servo[0].angle = 90 #X
kit.servo[4].angle = 90 #Y

model = YOLO("yolov8n.pt")
app = Flask(__name__)
net = cv.dnn.readNet('yolov4-tiny.weights', 'yolov4-tiny.cfg')
with open('coco.names', 'r') as f:
    classes = f.read().strip().split('\n')
frame = None
cap = None
frameweb = None
coms = {
    "Goto":None,
    "Point":None,
    "Mode":0,
    "SPDL": 0.8,
    "SPDR": 0.8,
}
led.value = 0
led2.value = 0
led3.value = 0
led4.value = 0

# websocket
async def handler(websocket):
    global coms,led,led2,led3,led4,camera_y,camera_x
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
                    kit.servo[4].angle = int(camera_y)
                elif data["status"] == "down":
                    camera_y -= 0.5
                    if camera_y < 0:
                        camera_y = 0
                    kit.servo[4].angle = int(camera_y)
                elif data["status"] == "left":
                    camera_x += 0.5
                    if camera_x > 180:
                        camera_x = 180
                    kit.servo[0].angle = int(camera_x)
                elif data["status"] == "right":
                    camera_x -= 0.5
                    if camera_x < 0:
                        camera_x = 0
                    kit.servo[0].angle = int(camera_x)
                elif data["status"] == "reset":
                    camera_x = 90
                    camera_y = 90
                    kit.servo[4].angle = int(camera_y)
                    kit.servo[0].angle = int(camera_x)
                elif data["status"] == "w":
                    #print("w")
                    coms["Point"] = 0.1
                    led.value = coms["SPDL"]
                    led2.value = 0
                    led3.value = 0
                    led4.value = coms["SPDR"]
                elif data["status"] == "s":
                    #print("s")
                    coms["Point"] = 0.1
                    led.value = 0
                    led2.value = coms["SPDL"]
                    led3.value = coms["SPDR"]
                    led4.value = 0
                elif data["status"] == "a":
                    #print("a")
                    coms["Point"] = 0.1
                    led.value = 0
                    led2.value = coms["SPDL"]/2
                    led3.value = 0
                    led4.value = coms["SPDR"]/2
                elif data["status"] == "d":
                    #print("d")
                    coms["Point"] = 0.1
                    led.value = coms["SPDL"]/2
                    led2.value = 0
                    led3.value = coms["SPDR"]/2
                    led4.value = 0
                elif data["status"] == "off":
                    led.value = 0
                    led2.value = 0
                    led3.value = 0
                    led4.value = 0
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



# Motor

def DriveMotor():
    global coms,led,led2,led3,led4
    while True:
        # print(f"SPDL = {coms['SPDL']} , SPDR = {coms['SPDR']}")
        if coms["Goto"] == "W":
            #print("GO")
            led.value = coms["SPDL"]
            led2.value = 0
            led3.value = 0
            led4.value = coms["SPDR"]
        elif coms["Goto"] == "D":
            led.value = coms["SPDL"]
            led2.value = 0
            led3.value = coms["SPDR"]
            led4.value = 0
        elif coms["Goto"] == "A":
            led.value = 0
            led2.value = coms["SPDL"]
            led3.value = 0
            led4.value = coms["SPDR"]
        elif coms["Goto"] == "S":
            led.value = 0
            led2.value = 0
            led3.value = 0
            led4.value = 0
        try:
            time.sleep(coms["Point"])
            coms["SPDR"] = coms["SPDR"] - 0.1
            coms["SPDL"] = coms["SPDL"] - 0.1
            if coms["SPDL"] < 0:
                coms["SPDL"] = 0
                
            if coms["SPDR"] < 0:
                coms["SPDR"] = 0           
        except:
            continue
        
        
# Yolo

def Yolo_tiny():
    global frame,net,frameweb,coms,camera_y,kit,camera_x
    timereset = 0
    lastcom = "D"
    modear = True
    while True:
        if frame is None or coms["Mode"] == 1:
            continue
    
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
        print(f"point {coms['Point']}")
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                confidence = confidences[i]
                color = (0, 255, 0)
                cv.rectangle(frameweb, (x, y), (x + w, y + h), color, 2)
                if label == "person":
                    print(f"x = {x} w = {w} x+w = {x+w}")
                    modear = False
                    if y < 100:
                        camera_y += 2
                        kit.servo[4].angle = int(camera_y)
                    print(((((1280 * 720 - w*h)/(1280 * 720))*100) / 1000))
                    print(100-(((1280 * 720 - w*h)/(1280 * 720))*100))
                    if  100-(((1280 * 720 - w*h)/(1280 * 720))*100) > 60:
                        print("person off")
                        break
                    if ((x+(w//2)) > 400 and (x+(w//2)) < 880) and coms["Mode"] == 0:
                        print("person W")
                        coms["Goto"] = "W"
                        coms["Point"] = 0.1 + ((((1280 * 720 - w*h)/(1280 * 720))*100) / 1000)
                        coms["SPDR"] = 1
                        coms["SPDL"] = 1
                    elif ((x+(w//2)) <= 400) and coms["Mode"] == 0:
                        print("person A")
                        coms["Goto"] = "A"
                        lastcom = "A"
                        coms["Point"] = (0.1 + ((400 - (x+w))/10000))
                        coms["SPDL"] = 0.35
                        coms["SPDR"] = 0.35
                    elif ((x+(w//2)) > 880) and coms["Mode"] == 0:
                        print("person D")
                        coms["Goto"] = "D"
                        lastcom = "D"
                        coms["Point"] = 0.1 + (((x+w) - 400)/10000)
                        coms["SPDL"] = 0.35
                        coms["SPDR"] = 0.35
                    timereset = 0
                    break
        while not camera_y == 95:
            if camera_y > 95:
                camera_y -= 1
                kit.servo[4].angle = int(camera_y)
            else:
                camera_y += 1
                kit.servo[4].angle = int(camera_y)
        if timereset > 7:
            # print(lastcom)
            coms["Goto"] = lastcom
            coms["Point"] = 0.15
            coms["SPDL"] = 0.5
            coms["SPDR"] = 0.5
            timereset = 0
            modear = True
        else:
            if coms["SPDL"] == 0 and coms["SPDR"] == 0 or modear:
                timereset += 1


def Yolo():
    mode = False
    tem = 0
    global model,frame,coms,camera_y,kit
    while True:
        if frame is None or coms["Mode"] == 1:
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
                kit.servo[4].angle = camera_y
                print(f"c = {a}")
                time.sleep(5)
                tem = 0
                continue
            if xy[0][3] > h - (h//15):
                camera_y = camera_y - 2
                if camera_y <= 0:
                    camera_y = 90
                kit.servo[4].angle = camera_y
                # time.sleep(0.5)
            elif not mode and tem == 0:
                tem = xy[0][3]
                camera_y = camera_y - 2
                if camera_y <= 0:
                    camera_y = 90
                kit.servo[4].angle = camera_y
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



# Start Programs

async def main_asyncio():
    await main()

  

if __name__ == "__main__":
    camera_thread = threading.Thread(target=read_camera)
    camera_thread.daemon = True
    Yolo_thread = threading.Thread(target=Yolo_tiny)
    Yolo_thread.daemon = True
    Motor_thread = threading.Thread(target=DriveMotor)
    Motor_thread.daemon = True
    camera_thread.start()
    Yolo_thread.start()
    Motor_thread.start()
    
    asyncio_thread = threading.Thread(target=asyncio.run, args=(main_asyncio(),))
    asyncio_thread.start()
    
    app.run(host='0.0.0.0', port=8000, threaded=True)
