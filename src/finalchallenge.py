import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
import time
import cv2 as cv
import numpy as np
from picamera2 import Picamera2 as pc2
direction=Nones
# Servo and GPIO setup
kit = ServoKit(channels=16)
IN1 = 17
IN2 = 27
ECHOF = 5
TRIGF = 6
ECHOR = 16
TRIGR = 26
ECHOL = 23
TRIGL = 24
turnAngle = 50
turnTime = 1.4
speed = 80
directionDetermined=False
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(TRIGF, GPIO.OUT)
GPIO.setup(ECHOF, GPIO.IN)
GPIO.setup(TRIGR, GPIO.OUT)
GPIO.setup(ECHOR, GPIO.IN)
GPIO.setup(TRIGL, GPIO.OUT)
GPIO.setup(ECHOL, GPIO.IN)
distancef=0
distancer=0
distancel=0
# PWM setup for movement
p = GPIO.PWM(IN1, 150)
pr = GPIO.PWM(IN2, 150)
p.start(speed)
GPIO.output(IN2, GPIO.LOW)
kit.servo[0].angle = 90
roundCount=0
obstacleCount=0

def object_detection(cam):
    frame = cam.capture_array()
    frame = cv.blur(frame, (8, 8))
    rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

    # Mask for green
    maskGreen = cv.inRange(rgb, (0, 100, 0), (100, 255, 100))

    # Mask for red
    maskRed = cv.inRange(rgb, (150, 0, 0), (255, 100, 100))

    # Morphological operations to remove small noise
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
    maskGreen = cv.morphologyEx(maskGreen, cv.MORPH_OPEN, kernel)
    maskRed = cv.morphologyEx(maskRed, cv.MORPH_OPEN, kernel)

    # Find contours for green
    contoursGreen, _ = cv.findContours(maskGreen, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if contoursGreen:
        largestGreenContour = max(contoursGreen, key=cv.contourArea)
    else:
        largestGreenContour = None

    # Find contours for red
    contoursRed, _ = cv.findContours(maskRed, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if contoursRed:
        largestRedContour = max(contoursRed, key=cv.contourArea)
    else:
        largestRedContour = None

    greenCount = cv.contourArea(largestGreenContour) if largestGreenContour is not None and cv.contourArea(largestGreenContour) > 5000 else 0
    redCount = cv.contourArea(largestRedContour) if largestRedContour is not None and cv.contourArea(largestRedContour) > 5000 else 0

    if redCount > greenCount:
        print("red")
    elif greenCount > redCount:
        print("green")

    # Draw contours and calculate position
    mask = np.zeros_like(frame)
    if largestGreenContour is not None and greenCount > 0:
        cv.drawContours(mask, [largestGreenContour], -1, (0, 255, 0), -1)
        M = cv.moments(largestGreenContour)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv.circle(mask, (cx, cy), 7, (255, 0, 0), -1)
            if cx < 640:
                return["g","l"]
            elif cx > 1280:
                return["g","r"]
            else:
                return["g","c"]

    if largestRedContour is not None and redCount > 0:
        cv.drawContours(mask, [largestRedContour], -1, (0, 0, 255), -1)
        M = cv.moments(largestRedContour)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv.circle(mask, (cx, cy), 7, (255, 0, 0), -1)
            if cx < 640:
                return["r","l"]
            elif cx > 1280:
                return["r","r"]
            else:
                return["r","c"]

    h, w, _ = mask.shape
    # Draw guide lines
    cv.line(mask, (w//2, h), (w//2, 0), (255, 0, 255), 10)
    cv.line(mask, (w//4, h), (w//4, 0), (255, 0, 255), 10)
    cv.line(mask, (w*3//4, h), (w*3//4, 0), (255, 0, 255), 10)
    cv.imshow('color', mask)

# Function to measure distance from sensors
def measure_distance(trigger, echo):
    GPIO.output(trigger, False)
    time.sleep(0.1)  # Ensure the trigger is off
    GPIO.output(trigger, True)
    time.sleep(0.00001)
    GPIO.output(trigger, False)

    pulse_start = time.time()
    pulse_end = time.time()

    while GPIO.input(echo) == 0:
        pulse_start = time.time()
    
    while GPIO.input(echo) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Convert to cm
    
    if distance > 1000 or distance < 2:  # Invalid readings
        return None
    return round(distance, 2)

# Function to handle movement based on sensor readings
def handle_movement(cam1):
    distancef = measure_distance(TRIGF, ECHOF)
    distancer = measure_distance(TRIGR, ECHOR)
    distancel = measure_distance(TRIGL, ECHOL)
    print("Front: " +str(distancef))
    print("Left: " +str(distancel))
    print("Right: "+str(distancer))

    camera=object_detection(cam1)
    # Logic for reversing if the front distance is too small
    if (distancef < 5 or distancef > 1000) or distancef is None:
        print("reverse")
        p.ChangeDutyCycle(0)
        kit.servo[0].angle = 180 - kit.servo[0].angle
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN1, GPIO.LOW)
        time.sleep(1.5)
        p.ChangeDutyCycle(speed)
        GPIO.output(IN2, GPIO.LOW)
    if camera[1]=="l":
        if camera[0]=="r":
            kit.servo[0].angle=90
        elif camera[0]=="g":
            kit.servo[0].angle=70
            while(measure_distance(TRIGL,ECHOL)>5):
                pass
            kit.servo[0].angle=90
    elif camera[1]=="r":
        if camera[0]=="g":
            kit.servo[0].angle=90
        elif camera[0]=="r":
            kit.servo[0].angle=110
            while(measure_distance(TRIGR,ECHOR)>5):
                pass
            kit.servo[0].angle=90
    
    # Logic for turning if front distance is below a certain threshold
    if distancef < 90:
        #if object detected is in center and coloured (not wall)
        if camera[1]=="c":
            if camera[0]=="r":
                kit.servo[0].angle=100
                p.ChangeDutyCycle(50)
                while(measure_distance(TRIGR,ECHOR)>5):
                    pass
                kit.servo[0].angle=90
                p.ChangeDutyCycle(speed)

            elif camera[0]=="g":
                kit.servo[0].angle=80
                p.ChangeDutyCycle(50)
                while(measure_distance(TRIGL,ECHOL)>5):
                    pass
                kit.servo[0].angle=90
                p.ChangeDutyCycle(speed)


        else:
            print("big turn")
            #follow given direction or take initial turn
            if (distancer is not None and distancel is not None and distancer > distancel) or direction=="r":
                if not directionDetermined:
                    direction="r"
                    directionDetermined=True
                kit.servo[0].angle = 90 + turnAngle
                p.ChangeDutyCycle(100)
                time.sleep(turnTime)
                p.ChangeDutyCycle(speed)
                kit.servo[0].angle = 90
                print("Turn right")

            elif (distancel is not None and distancer is not None  and distancel > distancer) or direction=="l":
                if not directionDetermined:
                    direction="l"
                    directionDetermined=True
                kit.servo[0].angle = 90 - turnAngle
                p.ChangeDutyCycle(100)
                time.sleep(turnTime)
                p.ChangeDutyCycle(speed)
                kit.servo[0].angle = 90
                print("Turn left")

    # self correction
    elif (distancer is not None and distancel is not None) and (distancer + distancel) < 110:
        if distancer > distancel:
            kit.servo[0].angle = 100
            print("adjust right")
        elif distancel > distancer:
            kit.servo[0].angle = 80
            print("adjust left")
        else:
            kit.servo[0].angle = 90



# Function for object detection using camera



cam = pc2()
cam.start()

try:
    while True:
        handle_movement(cam)


except KeyboardInterrupt:
    print("Program Ended by User")
    GPIO.cleanup()
    cam.close()
    cv.destroyAllWindows()
