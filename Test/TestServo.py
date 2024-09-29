from adafruit_servokit import ServoKit
import board
import busio
import time
i2c = busio.I2C(1, 0)
kit = ServoKit(i2c=i2c,channels=8)
camera_x = 0
camera_y = 0
temp = 0
kit.servo[0].angle = 90
while True:
    com = input()
    if com == "off":
        for i in range(90,0,-1):
            kit.servo[4].angle = i
            time.sleep(0.01)
    elif com == "on":
        for i in range(0,90):
            kit.servo[4].angle = i
            time.sleep(0.01)

# while True:
#     com = input()
#     if com == "on":
#         kit.servo[4].angle = 80
#         time.sleep(0.4)
#         kit.servo[4].angle = 90
#     elif com == "off":
#         kit.servo[4].angle = 10
#         time.sleep(0.4)
#         kit.servo[4].angle = 0
        
         
