    ,PWMLED

led = PWMLED(27) # br
led2 = PWMLED(22) #fr
led3 = PWMLED(4) #bl
led4 = PWMLED(17) #fl

while True:
    led.value = 0
    led2.value = 0
    led3.value = 0
    led4.value = 0