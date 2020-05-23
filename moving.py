import RPi.GPIO as GPIO
import time

print("Testing the Raspberry pi GPIO module")

def blink(pin, delay, times=1):
    # State can be 0/GPIO.LOW/False or 1/GPIO.HIGH/True
    for i in range(1, times+1):
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(pin, GPIO.LOW)
        time.sleep(delay)
    return

def doPWM(pin, frec):
    # Para generar PWM (pin, frecuency)
    pwm = GPIO.PWM(pin, frec)
    pwm.start(0)
    for i in range(1, 11):
        pwm.ChangeDutyCycle(i*10)
        time.sleep(0.5)
    # Remember to use pwm.stop()
    pwm.stop()
    return

def doServo(pin):
    servo = GPIO.PWM(pin, 50)
    servo.start(2)
    for i in range(1, 19):
        duty = angle2dutyCycle(i*10)
        servo.ChangeDutyCycle(duty)
        time.sleep(0.2)
    # Remember to use servo.stop()
    servo.stop()
    return

def angle2dutyCycle(angle):
    duty = 2 + (angle/18)
    return duty

def main():
    # modo BOARD: como se numeran en la placa
    GPIO.setmode(GPIO.BOARD)
    pin_Digital = 7
    pin_PWM     = 11
    pin_Servo   = 13
    GPIO.setup(pin_Digital, GPIO.OUT, initial = 0)    
    GPIO.setup(pin_PWM,     GPIO.OUT, initial = 0)
    GPIO.setup(pin_Servo,   GPIO.OUT, initial = 0)

    while True:
        blink(pin_Digital, 0.5, 3)
        doPWM(pin_PWM, 1000)
        doServo(pin_Servo) 

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("\nProgram finished")
