import cv2
import numpy as np
import time
import serial

video_path = "vid6.mp4"

setpoint = 320

threshold = 100000

# Initialize states for ROI detections
state_p1 = False
state_p2 = False
state_p3 = False
state_p4 = False

# Initialize command
cmd = ''

# Initialize serial communication
#ser = serial.Serial('COM9', 9600)  # Change 'COM3' to your Arduino's port
#time.sleep(2)  # Wait for the Arduino to initialize

# Function to detect white lines with erosion followed by dilation
def detect_white_lines(roi):
        # Convert ROI to grayscale
    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    
    # Threshold to detect white lines
    _, thresh = cv2.threshold(gray_roi, 180, 255, cv2.THRESH_BINARY)
    
    # Define erosion kernel
    kernel = np.ones((3, 3), np.uint8)
    
    # Perform erosion operation
    white_lines = cv2.erode(thresh, kernel, iterations=2)
    
    # Perform dilation operation
    white_lines = cv2.dilate(white_lines, kernel, iterations=9)
    
    return white_lines

# Capture video from file
cap = cv2.VideoCapture(1)

f = 0
error = 0

while True:
    cmd = ''
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
    frame = cv2.rotate(frame,cv2.ROTATE_180)
    frame_copy1 = frame.copy()
    frame_copy2 = frame.copy()

    # Create ROI (Region of Interest)
    roi1 = frame_copy1[0:120, 0:639]
    roi2 = frame_copy1[120:240, 0:639]
    roi3 = frame_copy1[240:360, 0:639]
    roi4 = frame_copy2[175:275, 0:639]

    # Detect white lines in each ROI
    white_lines1 = detect_white_lines(roi1)
    white_lines2 = detect_white_lines(roi2)
    white_lines3 = detect_white_lines(roi3)
    white_lines4 = detect_white_lines(roi4)

    # Find contours of the black lines
    contours, _ = cv2.findContours(white_lines4.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # If contours are found
    if len(contours) > 0:
        # Get the bounding rectangle of the largest contour
        x, y, w, h = cv2.boundingRect(contours[0])
        
        # Draw a vertical centerline through the bounding rectangle
        cv2.line(frame, (x + int(w / 2), 200), (x + int(w / 2), 250), (255, 0, 0), 3)
        
        # Calculate error between centerline and setpoint
        center_line = x + int(w / 2)
        error = center_line - setpoint
        if error < -200 and f < 3:
            error = 0
            f+=1
        

        
        cmd = str(error) + ',' + '140,0' + "\r"
    

        # Display error on the image
        centertext = "Error = " +  str(error)
        cv2.putText(frame, centertext, (200, 340), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 3)

    # Calculate pixel summation along columns (vertical summation)
    pixel_sum1 = np.sum(white_lines1, axis=0)
    pixel_sum2 = np.sum(white_lines2, axis=0)
    pixel_sum3 = np.sum(white_lines3, axis=0)

    # Check if the state of each region is "high" based on the threshold
   # print(np.sum(white_lines1))
   # print(np.sum(white_lines2))
   # print(np.sum(white_lines3))
   # print(np.sum(white_lines4))

 
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
    
    



    # Display the frame and command
    cv2.imshow('Centerline Detection', frame)
    cv2.imshow('white_lines1', white_lines1)
    cv2.imshow('white_lines2',white_lines2)
    cv2.imshow('white_lines3', white_lines3)
    cv2.imshow('white_lines4', white_lines4)
    cv2.imshow('Region of Interest',roi4 )

    print("Cmd: ",cmd)

    #ser.write(str(cmd).encode())

    # Check for key press
    key = cv2.waitKey(25)
    if key == 27:  # Press 'Esc' to exit
        break

# Release video capture object and close all windows
cap.release()
cv2.destroyAllWindows()
