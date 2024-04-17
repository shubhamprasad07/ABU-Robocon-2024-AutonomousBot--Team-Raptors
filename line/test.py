import cv2
import numpy as np
import time
import serial

# Video file path
video_path = "vid6.mp4"

# Setpoint for the centerline
setpoint = 320

# Threshold for considering the state of a region as "high"
threshold = 100000  # You may need to adjust this value based on your video

# Initialize serial communication
#ser = serial.Serial('COM9', 9600)  # Change 'COM9' to your Arduino's port

#time.sleep(2)  # Wait for the Arduino to initialize


# Initialize states for ROI detections
state_p1 = False
state_p2 = False
state_p3 = False
state_p4 = False

# Initialize counters for left and right turns
left_turn_count = 0
right_turn_count = 0

# Initialize command
cmd = ''

# Function to detect white lines with erosion followed by dilation

def detect_white_lines(roi):
    # Convert ROI to HSV color space
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Define lower and upper bounds for white and yellow colors in HSV
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 25, 255])
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    
    # Create masks for white and yellow regions
    mask_white = cv2.inRange(hsv_roi, lower_white, upper_white)
    mask_yellow = cv2.inRange(hsv_roi, lower_yellow, upper_yellow)
    
    # Combine masks to detect both white and yellow regions
    combined_mask = cv2.bitwise_or(mask_white, mask_yellow)
    
    # Perform morphological operations to enhance white lines
    kernel = np.ones((3, 3), np.uint8)
    white_lines = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
    
    return white_lines

# Capture video from file
cap = cv2.VideoCapture(video_path)

while True:
    # Read frame from video
    ret, frame = cap.read()
    
    # Check if video has ended
    if not ret:
        # Reload the video
        cap.release()
        cap = cv2.VideoCapture(video_path)
        continue
    
    # Resize the frame to 640x360
    frame = cv2.resize(frame, (640, 360))
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    frame_copy = frame.copy()
    
    # Create ROIs (Regions of Interest)
    roi_height = 120
    roi_width = 640
    roi1 = frame_copy[0:roi_height, 0:roi_width]
    roi2 = frame_copy[120:240, 0:140]
    roi3 = frame_copy[240:360, 0:640]
    roi4 = frame_copy[120:240, 540:640]
    
    # Detect white lines in each ROI
    white_lines1 = detect_white_lines(roi1)
    white_lines2 = detect_white_lines(roi2)
    white_lines3 = detect_white_lines(roi3)
    white_lines4 = detect_white_lines(roi4)
    
    # Calculate pixel summation along columns (vertical summation)
    pixel_sum1 = np.sum(white_lines1, axis=0)
    pixel_sum2 = np.sum(white_lines2, axis=0)
    pixel_sum3 = np.sum(white_lines3, axis=0)
    
    # Check if the state of each region is "high" based on the threshold
    if np.sum(white_lines1) > threshold:
        state_p1 = True
    else:
        state_p1 = False
    
    if np.sum(white_lines2) > threshold:
        state_p2 = True
    else:
        state_p2 = False
    
    if np.sum(white_lines3) > threshold:
        state_p3 = True
    else:
        state_p3 = False
    
    if np.sum(white_lines2) > threshold:
        state_p4 = True
    else:
        state_p4 = False

    # Update command based on the states of p1, p2, p3, and p4
    #if state_p1 and state_p2 and state_p3 and state_p4:
        #cmd = 'S'
    if state_p1 and state_p4:
        cmd = 'f'
    elif state_p1 and state_p3:
        cmd = 'l'
    elif not state_p1 and state_p4:
        cmd = 'r'

    elif state_p1 and state_p2 and state_p3 and state_p4:
        break
    
    # Decide left and right turns based on the states of p2 and p4
    if state_p2 and not state_p4:
        cv2.putText(frame, 'Left Turn', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        left_turn_count += 1
    elif state_p4 and not state_p2:
        cv2.putText(frame, 'Right Turn', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        right_turn_count += 1
    
    # Display the frame and command
    cv2.imshow('Turn Detection', frame)
    cv2.imshow('Whitelines1', white_lines1)
    cv2.imshow('Whitelines2', white_lines2)
    cv2.imshow('Whitelines3', white_lines3)
    cv2.imshow('Whitelines4', white_lines4)
    print("cmd:", cmd)
    cmd = cmd + '\r'
    #ser.write(str(cmd).encode())
    
    # Check for key press
    key = cv2.waitKey(25)
    if key == 27:  # Press 'Esc' to exit
        break

# Release video capture object and close all windows
cap.release()
cv2.destroyAllWindows()
