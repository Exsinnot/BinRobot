from gpiozero import LED,PWMLED,DistanceSensor

led = PWMLED(27) # br
led2 = PWMLED(22) #fr
led3 = PWMLED(17) #bl
led4 = PWMLED(4) #fl

while True:
    led.value = 0
    led2.value = 0.5
    led3.value = 0.5
    led4.value = 0