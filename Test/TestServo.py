from adafruit_servokit import ServoKit
kit = ServoKit(channels=8)
camera_x = 90
camera_y = 0
kit.servo[0].angle = camera_x #X
kit.servo[4].angle = camera_y #Y