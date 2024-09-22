import time

import board
import busio

import adafruit_vl53l0x
from digitalio import DigitalInOut
from adafruit_vl53l0x import VL53L0X

i2c = board.I2C()
xshut = [
    DigitalInOut(board.D20), 
    DigitalInOut(board.D21) 
]
for power_pin in xshut:
    power_pin.switch_to_output(value=True)

vl53 = []
xshut[0].value = True 
xshut[1].value = False
# xshut[0].value = True 
# time.sleep(0.1)       
# vl53.append(VL53L0X(i2c)) 
# vl53[0].set_address(0x2B)  
# xshut[0].value = False
# xshut[1].value = True 
# time.sleep(0.1)       
# vl53.append(VL53L0X(i2c)) 
# vl53[1].set_address(0x2A) 


# xshut[0].value = True 
# xshut[1].value = True  

# i2c = busio.I2C(board.SCL, board.SDA)
# vl53 = adafruit_vl53l0x.VL53L0X(i2c=i2c,address=41)
# vl532 = adafruit_vl53l0x.VL53L0X(i2c=i2c,address=0x2a)

# vl53.measurement_timing_budget = 100000
# vl532.measurement_timing_budget = 100000

# while True:
#     try:
#         print("Range1: {0}cm".format(vl53.range/10))
#         print("Range2: {0}cm".format(vl532.range/10))
#         time.sleep(1.0)
#     except:
#         print("EEROR")