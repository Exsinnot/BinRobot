import RPi.GPIO as GPIO
pwm_br = None
pwm_fr = None
pwm_bl = None
pwm_fl = None
def SetUp4Pin(pin_fr,pin_br,pin_fl,pin_bl):
    global pwm_bl,pwm_br,pwm_fl,pwm_fr
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin_br, GPIO.OUT)
    GPIO.setup(pin_fr, GPIO.OUT)
    GPIO.setup(pin_bl, GPIO.OUT)
    GPIO.setup(pin_fl, GPIO.OUT)
    
    pwm_br = GPIO.PWM(pin_br, 100)  
    pwm_fr = GPIO.PWM(pin_fr, 100)
    pwm_bl = GPIO.PWM(pin_bl, 100)
    pwm_fl = GPIO.PWM(pin_fl, 100)

    pwm_br.start(0)
    pwm_fr.start(0)
    pwm_bl.start(0)
    pwm_fl.start(0)
    print("Setup succ")

def W(Speed):
    global pwm_bl,pwm_br,pwm_fl,pwm_fr
    pwm_br.ChangeDutyCycle(0)              
    pwm_fr.ChangeDutyCycle(Speed)      
    pwm_bl.ChangeDutyCycle(0)              
    pwm_fl.ChangeDutyCycle(Speed)   
def S(Speed):
    global pwm_bl,pwm_br,pwm_fl,pwm_fr
    pwm_br.ChangeDutyCycle(Speed)    
    pwm_fr.ChangeDutyCycle(0)   
    pwm_bl.ChangeDutyCycle(Speed)   
    pwm_fl.ChangeDutyCycle(0)  
def A(Speed,Degee = None):
    global pwm_bl,pwm_br,pwm_fl,pwm_fr
    pwm_br.ChangeDutyCycle(0)    
    pwm_fr.ChangeDutyCycle(Speed)   
    pwm_bl.ChangeDutyCycle(Speed)   
    pwm_fl.ChangeDutyCycle(0)  
def D(Speed,Degee = None):
    global pwm_bl,pwm_br,pwm_fl,pwm_fr
    pwm_br.ChangeDutyCycle(Speed)    
    pwm_fr.ChangeDutyCycle(0)   
    pwm_bl.ChangeDutyCycle(0)   
    pwm_fl.ChangeDutyCycle(Speed)  
def Stop():
    global pwm_bl,pwm_br,pwm_fl,pwm_fr
    pwm_br.ChangeDutyCycle(0)    
    pwm_fr.ChangeDutyCycle(0)   
    pwm_bl.ChangeDutyCycle(0)   
    pwm_fl.ChangeDutyCycle(0)  
