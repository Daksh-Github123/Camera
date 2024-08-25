import cv2
import numpy as np
import time

cap = cv2.VideoCapture(0)

# Check camera open
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Define color ranges in HSV
lower_blue = np.array([94, 80, 2])
upper_blue = np.array([126, 255, 255])

lower_orange = np.array([10, 100, 20])
upper_orange = np.array([25, 255, 255])

lower_green = np.array([40, 40, 40])
upper_green = np.array([70, 255, 255])

lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 120, 70])
upper_red2 = np.array([180, 255, 255])

# Define the minimum contour area
MIN_CONTOUR_AREA = 750  # Adjust this value as needed

while True:
    # Capture frame-by-frame from the laptop camera
    ret, frame = cap.read()
    if not ret:
        print("Error: Couldn't retrieve frame.")
        break

    # Convert frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create masks for the colors
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.add(mask_red1, mask_red2)

    # Find contours for each color
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Check for blue line with size condition
    if any(cv2.contourArea(c) > MIN_CONTOUR_AREA for c in contours_blue):
        print("Blue line detected")
        time.sleep(2)
        # TODO: Implement action when blue line is detected

    # Check for orange line with size condition
    if any(cv2.contourArea(c) > MIN_CONTOUR_AREA for c in contours_orange):
        print("Orange line detected")
        time.sleep(2)
        # TODO: Implement action when orange line is detected

    # Check for green object with size condition
    if any(cv2.contourArea(c) > MIN_CONTOUR_AREA for c in contours_green):
        print("Green object detected")
        time.sleep(2)
        # TODO: Implement action when green object is detected

    # Check for red object with size condition
    if any(cv2.contourArea(c) > MIN_CONTOUR_AREA for c in contours_red):
        print("Red object detected")
        time.sleep(2)
        # TODO: Implement action when red object is detected

    # Display the frame
    cv2.imshow('Frame', frame)

    # Break loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close windows
cap.release()
cv2.destroyAllWindows()
