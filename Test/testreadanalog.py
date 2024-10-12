import RPi.GPIO as GPIO
import time

# Pin setup
button_pin = 25
GPIO.setmode(GPIO.BCM)
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Using pull-down resistor

# Read pin state
try:
    while True:
        print(GPIO.input(button_pin))
        if GPIO.input(button_pin):
            print("Button pressed!")
        else:
            print("Button not pressed!")
        time.sleep(0.5)
except KeyboardInterrupt:
    GPIO.cleanup()
