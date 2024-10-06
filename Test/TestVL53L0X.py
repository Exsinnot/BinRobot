import time
import RPi.GPIO as GPIO
import board
import busio
from digitalio import DigitalInOut
from adafruit_vl53l0x import VL53L0X
i2c = busio.I2C(1, 0)
# xshut = [
#     DigitalInOut(board.D20), 
#     DigitalInOut(board.D21) 
# ]
# for power_pin in xshut:
#     power_pin.switch_to_output(value=False)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.output(20,0)
GPIO.output(21,0)

GPIO.output(20,1)
GPIO.output(21,0)
time.sleep(0.1)       
VL53L0X(i2c,address=41).set_address(0x29)
time.sleep(0.1)       
GPIO.output(20,0)
GPIO.output(21,1)
VL53L0X(i2c,address=41).set_address(0x2A)
GPIO.output(20,1)
GPIO.output(21,1)

vl53 = VL53L0X(i2c=i2c,address=41)
vl532 = VL53L0X(i2c=i2c,address=0x2a)

vl53.measurement_timing_budget = 50000
vl532.measurement_timing_budget = 50000
vl53.signal_rate_limit = 0.1
vl532.signal_rate_limit = 0.1


while True:
    try:
        distance1 = vl53.range
        distance2 = vl532.range

        print("Range1: {0}cm".format(distance1/10))
        print("Range2: {0}cm".format(distance2/10))
        time.sleep(0.1)
    except KeyboardInterrupt:
        break