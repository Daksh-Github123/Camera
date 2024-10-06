import cv2
import numpy as np
from adafruit_servokit import ServoKit
import time
from math import isclose
import RPi.GPIO as GPIO
from rplidar import RPLidar, RPLidarException
from mpu6050 import mpu6050

# Initialize
kit = ServoKit(channels=16)
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(PORT_NAME)
speed = 50
turnAngle = 65
obstacleThreshold = 2000

# Servo angles and motor pins
smallTurnAngleLeft = 30
smallTurnAngleRight = 150
centerAngle = 90
STEERING_CHANNEL = 0
STEERING_LEFT = 75
STEERING_RIGHT = 105
STEERING_STRAIGHT = 90

MOTOR_PIN_FORWARD = 17
MOTOR_PIN_BACKWARD = 27

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PIN_FORWARD, GPIO.OUT)
GPIO.setup(MOTOR_PIN_BACKWARD, GPIO.OUT)
p = GPIO.PWM(MOTOR_PIN_FORWARD, 150)


# Initialize video capture
cam = cv2.VideoCapture(0)

# Global variables
currentSign = "none"
obstacleState = "none"
startTime = None
lastTurn = None
turnDir = "none"
run = True
counter = 0

# Functions for motor and servo control
def stop():
    GPIO.output(MOTOR_PIN_FORWARD, GPIO.LOW)
    GPIO.output(MOTOR_PIN_BACKWARD, GPIO.LOW)

def steer_left():
    kit.servo[STEERING_CHANNEL].angle = STEERING_LEFT

def steer_right():
    kit.servo[STEERING_CHANNEL].angle = STEERING_RIGHT

def steer_straight():
    kit.servo[STEERING_CHANNEL].angle = STEERING_STRAIGHT

def subtle_left():
    p.ChangeDutyCycle(speed+20)
    kit.servo[0].angle = smallTurnAngleLeft
    time.sleep(1)
    kit.servo[0].angle = centerAngle

def subtle_right():
    p.ChangeDutyCycle(speed+20)
    kit.servo[0].angle = smallTurnAngleRight
    time.sleep(0.5)
    kit.servo[0].angle = centerAngle

def return_right(boundaries):
    print(boundaries['left'], boundaries['right'])
    kit.servo[0].angle = smallTurnAngleRight
    if(isclose(boundaries['left'],boundaries['right'],abs_tol=20)):
        kit.servo[0].angle=centerAngle
        p.ChangeDutyCycle(speed)
        return True
        
    else:
        return False
    
def return_left(boundaries):
    kit.servo[0].angle = smallTurnAngleLeft
    if(isclose(boundaries['left'],boundaries['right'],abs_tol=20)):
        kit.servo[0].angle=centerAngle
        p.ChangeDutyCycle(speed)
        return True
    else:
        return False
    
def maintain_center():
    kit.servo[0].angle = centerAngle

# LIDAR functions
def process_scan(scan):
    valid_measurements = []
    for item in scan:
        if len(item) >= 3 and item[2] > 0:
            angle, distance = item[1], item[2]
            valid_measurements.append((angle, distance))
    return valid_measurements

def find_boundaries(measurements):
    sectors = {'front': [], 'left': [], 'right': []}
    for measurement in measurements:
        try:
            angle, distance = measurement
        except ValueError as e:
            continue

        angle = angle % 360
        if 330 <= angle or angle <= 30:
            sectors['front'].append(distance)
        elif 30 < angle <= 150:
            sectors['left'].append(distance)
        elif 210 <= angle < 330:
            sectors['right'].append(distance)

    boundaries = {}
    for sector in sectors:
        boundaries[sector] = min(sectors[sector]) if sectors[sector] else None
    return boundaries

# Object detection using camera (Green/Red Sign Detection)
def object_detection(cam):
    ret, frame = cam.read()
    frame=cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    frame = cv2.blur(frame, (8, 8))
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Mask for green
    maskGreen = cv2.inRange(rgb, (0, 100, 0), (100, 255, 100))

    # Mask for red
    maskRed = cv2.inRange(rgb, (150, 0, 0), (255, 100, 100))

    # Morphological operations to remove small noise
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    maskGreen = cv2.morphologyEx(maskGreen, cv2.MORPH_OPEN, kernel)
    maskRed = cv2.morphologyEx(maskRed, cv2.MORPH_OPEN, kernel)

    # Find contours for green
    contoursGreen, _ = cv2.findContours(maskGreen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contoursGreen:
        largestGreenContour = max(contoursGreen, key=cv2.contourArea)
    else:
        largestGreenContour = None

    # Find contours for red
    contoursRed, _ = cv2.findContours(maskRed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contoursRed:
        largestRedContour = max(contoursRed, key=cv2.contourArea)
    else:
        largestRedContour = None

    greenCount = cv2.contourArea(largestGreenContour) if largestGreenContour is not None and cv2.contourArea(largestGreenContour) > 5000 else 0
    redCount = cv2.contourArea(largestRedContour) if largestRedContour is not None and cv2.contourArea(largestRedContour) > 5000 else 0

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Threshold to detect black areas
    _, threshold = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

    # Calculate the percentage of black pixels in the frame
    black_pixels = np.sum(threshold == 255)
    total_pixels = threshold.size
    black_percentage = (black_pixels / total_pixels) * 100

    # Check if more than 50% of the frame is black
    if black_percentage > 50:
        return ["b", "b"]

    # Draw contours and calculate position
    mask = np.zeros_like(frame)
    print(redCount > greenCount)
    if greenCount > 0 and greenCount > redCount:
        cv2.drawContours(mask, [largestGreenContour], -1, (0, 255, 0), -1)
        M = cv2.moments(largestGreenContour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(mask, (cx, cy), 7, (255, 0, 0), -1)
            if cx < 640:
                return "green", "right"
            elif cx > 1280:
                return "green", "left"
            else:
                return "green", "center"

    elif redCount > 0 and redCount > greenCount:
        cv2.drawContours(mask, [largestRedContour], -1, (0, 0, 255), -1)
        M = cv2.moments(largestRedContour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(mask, (cx, cy), 7, (255, 0, 0), -1)
            if cx < 640:
                return "red", "left"
            elif cx > 1280:
                return "red", "right"
            else:
                return "red", "center"

    else:
        return ["n", "n"]

# Function to process the frame for blue/orange detection and adjust movement
def process_frame(frame, distancef, distancel, distancer):
    global lastTurn, counter, turnDir
    print(f"Front: {distancef}, Left: {distancel}, Right: {distancer}")

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Threshold to detect black walls (adjust threshold value if needed)
    _, threshold = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

    # Threshold for Colored Line Detection
    frame1 = cv2.blur(frame, (8, 8))
    hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)

    # Blue Boundaries
    blue_lower = np.array([100, 150, 0])
    blue_upper = np.array([140, 255, 255])

    # Orange Boundaries
    orange_lower = np.array([10, 100, 20])
    orange_upper = np.array([25, 255, 255])

    # Color Masks
    mask_blue = cv2.inRange(hsv, blue_lower, blue_upper)
    mask_orange = cv2.inRange(hsv, orange_lower, orange_upper)

    # Find contours for blue and orange colors
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Logic for blue and orange detection, handling turns
    if len(contours_blue) > 0 and time.time() - lastTurn > 1.8:
        counter += 1
        p.ChangeDutyCycle(speed // 1)
        kit.servo[0].angle = 90 - turnAngle
        turnDir = "left"

        # Simulate turn
        lidar.disconnect()
        time.sleep(0.7)
        lidar.connect()
        print('Starting Left Turn (Blue)')
        time.sleep(1.5)
        print("Ending Left Turn")
        kit.servo[0].angle = 90
        p.ChangeDutyCycle(speed)
        turnDir = "final"
        lastTurn = time.time()

    elif len(contours_orange) > 0 and time.time() - lastTurn > 1.8:
        counter += 1
        p.ChangeDutyCycle(speed // 1)
        kit.servo[0].angle = 90 + turnAngle
        turnDir = "right"

        # Simulate turn
        lidar.disconnect()
        time.sleep(0.7)
        lidar.connect()
        print('Starting Right Turn (Orange)')
        time.sleep(1.5)
        print("Ending Right Turn")
        kit.servo[0].angle = 90
        p.ChangeDutyCycle(speed)
        turnDir = "final"
        lastTurn = time.time()

    if turnDir == "final":
        if distancer is None or distancel is None or distancef is None:
            turnDir = "none"
        
# Handling movement based on object detection and LIDAR
def handle_movement(cam, boundaries):
    global currentSign, startTime, obstacleState
    distancef, distancel, distancer = boundaries['front'], boundaries['left'], boundaries['right']

    sign_color, direction = object_detection(cam)

    if sign_color == "green" and direction!="right":
        if obstacleState == "none":
            # Slightly reverse before anything else happens
            p.ChangeDutyCycle(speed // 2)
            GPIO.output(MOTOR_PIN_BACKWARD, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(MOTOR_PIN_BACKWARD, GPIO.LOW)
            p.ChangeDutyCycle(speed)
            currentSign = "green"
            obstacleState = "green"
        print("Green Detected, avoiding")
        subtle_left()

    elif sign_color == "red" and direction!="left":
        if obstacleState == "none":
            p.ChangeDutyCycle(speed // 2)
            GPIO.output(MOTOR_PIN_BACKWARD, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(MOTOR_PIN_BACKWARD, GPIO.LOW)
            p.ChangeDutyCycle(speed)
            currentSign = "red"
            obstacleState = "red"
        print("Red Detected, avoiding")
        subtle_right()

    else:
        if currentSign == "green":
            if return_right(boundaries):
                currentSign = "none"
        elif currentSign == "red":
            if return_left(boundaries):
                currentSign = "none"
        else:
             maintain_center()

        if distancer is not None and distancel is not None:
            if distancer < distancel - 15:
                steer_left()
            elif distancer > distancel + 15:
                steer_right()

        # Adjust if too close to obstacles on the left or right
        if distancel is not None and distancel < 20:
            steer_right()
        elif distancer is not None and distancer < 20:
            steer_left()

    # Adjust based on LIDAR


# Main loop
try:
    steer_straight()
    lastTurn = time.time()

    while run:
        try:
            for scan in lidar.iter_scans():
                measurements = process_scan(scan)
                boundaries = find_boundaries(measurements)
                ret, frame = cam.read()
                process_frame(frame, boundaries['front'], boundaries['left'], boundaries['right'])
                handle_movement(cam, boundaries)

        except RPLidarException as e:
            p.ChangeDutyCycle(speed // 2)
            lidar.clean_input()
            lidar.stop()
            lidar.disconnect()
            time.sleep(0.7)
            p.start(speed)
            lidar.connect()
            lidar.start_motor()
            lastTurn = time.time()
        except ValueError:
            pass

except KeyboardInterrupt:
    print("Program interrupted by user")
finally:
    cam.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
