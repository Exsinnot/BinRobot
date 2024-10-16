from gpiozero import DistanceSensor
import time

sensor1 = DistanceSensor(echo=11, trigger=9,max_distance=8) #IN
sensor2 = DistanceSensor(echo=24, trigger=23,max_distance=8) #F
sensor3 = DistanceSensor(echo=14, trigger=15,max_distance=8) #R
sensor4 = DistanceSensor(echo=6, trigger=13,max_distance=8) #F

try:
    while True:
        dis = sensor1.distance * 100 
        print('Distance 1 : {:.2f} cm'.format(dis))  
        time.sleep(0.1)  
        dis = sensor2.distance * 100 
        print('Distance 1 : {:.2f} cm'.format(dis))  
        time.sleep(0.1)  
        dis = sensor3.distance * 100 
        print('Distance 1 : {:.2f} cm'.format(dis))  
        time.sleep(0.1)  
        dis = sensor4.distance * 100 
        print('Distance 1 : {:.2f} cm'.format(dis))  
        time.sleep(0.1)  
        time.sleep(2)
except KeyboardInterrupt:
    pass