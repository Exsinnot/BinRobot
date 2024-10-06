import RPi.GPIO as GPIO
import time

# กำหนดพิน GPIO
pin_br = 27  # br
pin_fr = 22  # fr
pin_bl = 17  # bl
pin_fl = 4   # fl

# ตั้งค่า GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_br, GPIO.OUT)
GPIO.setup(pin_fr, GPIO.OUT)
GPIO.setup(pin_bl, GPIO.OUT)
GPIO.setup(pin_fl, GPIO.OUT)

# ตั้งค่า PWM ที่แต่ละพิน
pwm_br = GPIO.PWM(pin_br, 1000)  # ความถี่ 100Hz
pwm_fr = GPIO.PWM(pin_fr, 1000)
pwm_bl = GPIO.PWM(pin_bl, 1000)
pwm_fl = GPIO.PWM(pin_fl, 1000)

# เริ่มต้น PWM ด้วยค่า duty cycle 0
pwm_br.start(0)
pwm_fr.start(0)
pwm_bl.start(0)
pwm_fl.start(0)

try:
    while True:
        pwm_br.ChangeDutyCycle(0)    # ปิดไฟ br
        pwm_fr.ChangeDutyCycle(30)   # เปิดไฟ fr 50%
        pwm_bl.ChangeDutyCycle(30)   # เปิดไฟ bl 50%
        pwm_fl.ChangeDutyCycle(0)    # ปิดไฟ fl
        time.sleep(1)

except KeyboardInterrupt:
    pass

finally:
    # หยุด PWM และคืนค่า GPIO
    pwm_br.stop()
    pwm_fr.stop()
    pwm_bl.stop()
    pwm_fl.stop()
    GPIO.cleanup()
