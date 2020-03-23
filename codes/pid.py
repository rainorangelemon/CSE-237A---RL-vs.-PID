import RPi.GPIO as GPIO
from calculate_angles import get_rotation
from time import sleep
from time import time

decay_ratio = 0.98
KP = 5
KI = 0
KD = 12.8565
DT = 0.035

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

state = prev_button = False

# y_gyro_bias, y_angle_bias = get_rotation()  # (1.83206106870229, -3.87594607384008)

try:

    while True:
        val = GPIO.input(11)
        if val == GPIO.LOW:
            if not prev_button:
                y_gyro_bias, y_angle_bias = get_rotation()
                # print(y_angle_bias)

                lastAngle = angle_Y = 0
                P_term = I_term = D_term = 0
                current_time = time()

                state = not state
                prev_button = True
        else:
            prev_button = False

        if not state:
            motor1.ChangeDutyCycle(0)
            motor2.ChangeDutyCycle(0)
            continue

        current_time = time()

        y_gyro, y_angle = get_rotation()
        y_angle -= y_angle_bias
        y_gyro -= y_gyro_bias
        angle_Y = decay_ratio * (angle_Y - y_gyro * DT) + (1 - decay_ratio) * y_angle
        # print(time() - current_time)
        # print(angle_Y)

        # PID
        P_term = KP * angle_Y
        I_term += KI * angle_Y
        D_term = KD * (angle_Y - lastAngle)
        lastAngle = angle_Y

        speed = P_term + I_term + D_term
        speed = min(max(speed, -100), 100)

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

        # print(speed)
        # print(time() - current_time)

        sleep(DT - (time() - current_time))

finally:
    GPIO.cleanup()
