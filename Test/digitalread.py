import RPi.GPIO as GPIO
import time

# Set up GPIO numbering
GPIO.setmode(GPIO.BCM)  # Use BCM numbering
pin = 16  # Replace with your GPIO pin number

# Set up the pin as an input with an optional pull-up resistor
GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Or PUD_DOWN
GPIO.setup(26, GPIO.OUT)  # Or PUD_DOWN


try:
    while True:
        GPIO.output(26,1)
        input_state = GPIO.input(pin)
        if input_state == GPIO.HIGH:
            print("Pin is HIGH")
        else:
            print("Pin is LOW")
        time.sleep(0.5)  # Adjust delay as needed

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()  # Clean up the GPIO state when done
