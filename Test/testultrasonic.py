from gpiozero import DistanceSensor
import time
sensor = DistanceSensor(echo=6, trigger=5,max_distance=8)
sensor2 = DistanceSensor(echo=16, trigger=26,max_distance=8)

try:
    while True:
        dis = sensor.distance * 100 
        print('Distance 1 : {:.2f} cm'.format(dis))  
        time.sleep(0.1)  
        dis = sensor2.distance * 100  
        print('Distance 2 : {:.2f} cm'.format(dis))  
        time.sleep(0.1)  
except KeyboardInterrupt:
    pass