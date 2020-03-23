import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)

GPIO.setup(0, GPIO.OUT)
GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)

GPIO.setup(11, GPIO.IN)  # button

GPIO.output(5, GPIO.HIGH)
GPIO.output(6, GPIO.LOW)
GPIO.output(13, GPIO.HIGH)
GPIO.output(19, GPIO.LOW)

motor1 = GPIO.PWM(0, 101)
motor2 = GPIO.PWM(26, 101)

motor1.start(0)
motor2.start(0)


def control_motor(choice):
    speed = 0 + 10 * choice
    if speed < 0:
        GPIO.output(5, GPIO.HIGH)
        GPIO.output(6, GPIO.LOW)
        GPIO.output(13, GPIO.HIGH)
        GPIO.output(19, GPIO.LOW)
    else:
        GPIO.output(5, GPIO.LOW)
        GPIO.output(6, GPIO.HIGH)
        GPIO.output(13, GPIO.LOW)
        GPIO.output(19, GPIO.HIGH)
    motor1.ChangeDutyCycle(abs(speed))
    motor2.ChangeDutyCycle(abs(speed))

try:
    for i in range(10):
        print(i)
        control_motor(i)
        sleep(1)


finally:
    GPIO.cleanup()
