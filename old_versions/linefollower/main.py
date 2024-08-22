#import libraries
import cv2
import numpy as np
import time
import serial

#initial parameters
Team = 'NA'
Zone = 'NA'
Turn = 'NA'
White_line_detection = 'NA'
Bot_State = 'NA'

P1 = 0
P2 = 0
P3 = 0

error = 0
cmd = str(error) + ',' + '140,0' + "\r"


#Functions:
#red color mask function
def red_mask(frame):
    # convert bgr to hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of red color in hsv
    lower_red = np.array([0,99,186])
    upper_red = np.array([0,255,255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_red, upper_red)
    kernel = np.ones((7,7), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=2)

    return mask

#blue color mask function
def blue_mask(frame):
    # convert bgr to hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of red color in hsv
    lower_blue = np.array([0,0,0])
    upper_blue = np.array([0,0,0])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    kernel = np.ones((7,7),np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=2)

    return mask

#white color mask function
def white_mask(roi):
    # Convert ROI to grayscale
    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # Threshold to detect white lines
    _, thresh = cv2.threshold(gray_roi, 197, 255, cv2.THRESH_BINARY)


    # Define erosion kernel
    kernel = np.ones((3, 3), np.uint8)

    # Perform erosion operation
    white_lines = cv2.erode(thresh, kernel, iterations=2)

    # Perform dilation operation
    white_lines = cv2.dilate(white_lines, kernel, iterations=9)

    return white_lines

#line follower function
def line_follower(frame,roi,White_line_detection): #parameters:- original frame and roi of whiteline for line follower
    setpoint = 320
    error = 0

    # Find contours of the black lines
    contours, _ = cv2.findContours(roi.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # If contours are found
    if len(contours) > 0:
        White_line_detection = 'Yes'
        # Get the bounding rectangle of the largest contour
        x, y, w, h = cv2.boundingRect(contours[0])
        
        # Draw a vertical centerline through the bounding rectangle
        cv2.line(frame, (x + int(w / 2), 200), (x + int(w / 2), 250), (255, 0, 0), 3)
        
        # Calculate error between centerline and setpoint
        center_line = x + int(w / 2)
        error = center_line - setpoint

        # Display error on the image
        centertext = "Error = " +  str(error)
        cv2.putText(frame, centertext, (200, 340), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 3)

        # Display regions of P2||P1||P3
        #P1 region
        cv2.rectangle(frame,(170,0),(490,120),(0,0,255),3)

        #P2 region
        cv2.rectangle(frame,(0,125),(170,250),(0,0,255),3)

        #P3 region
        cv2.rectangle(frame,(490,125),(639,250),(0,0,255),3)

        #line follower error region
        cv2.rectangle(frame,(0,125),(639,250),(0,255,0),3)

    return error,White_line_detection

#Rotate function
def rotate(Turn):
    cmd = ""
    if Turn == 'Left':
        cmd = 'L' + ',' + '140,0' + "\r"
    elif Turn == 'Right':
        cmd = 'R' + ',' + '140,0' + "\r"
    return cmd

#Cross-section detection function
def Cross_section_detection(P1,P2,P3,error):
    error = error
    cmd = str(error) + ',' + '140,0' + "\r"
    State = 'Running'
    if P2 > 100000 and P1 > 1000000 and P3 > 100000:
        cmd = 'S' + ',' + '140,0' + "\r"
        State = 'Stop'
    return cmd,State

#Initializing serial communication
#ser = serial.Serial('COM11', 9600)
#time.sleep(2)

#import video path+
video_path = "vid3.mp4"

#Capture video from file
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

    # Resize the frame to 640x360)
    frame = cv2.resize(frame, (640, 360))
    
    # Rotate the frame by 180
    #frame = cv2.rotate(frame,cv2.ROTATE_180)

    # Create copy of main frame
    frame_copy1 = frame.copy()
    frame_copy2 = frame.copy()
    frame_copy3 = frame.copy()

    # Create ROI (Region of Interest)
    roi1 = frame_copy1[0:120, 170:490]
    roi2 = frame_copy1[125:250, 0:170]
    roi3 = frame_copy1[125:250, 490:639]
    roi4 = frame_copy2[175:275, 0:639]
    roi5 = frame_copy3[0:360, 0:639]

    #red mask call
    r_mask = red_mask(frame)

    #blue mask call
    b_mask = blue_mask(frame)

    #P1 mask
    P1_mask = white_mask(roi1)

    #P2 mask
    P2_mask = white_mask(roi2)

    #P3 mask
    P3_mask = white_mask(roi3)

    #Line following mask
    Line_mask = white_mask(roi4)

    #white mask for full screen
    m = white_mask(roi5)

    #Calculation of pixel summation 
    P1 = np.sum(P1_mask)
    P2 = np.sum(P2_mask)
    P3 = np.sum(P3_mask)

    #Logical Conditions or line following algorithm
    if np.any(r_mask == 255):
        Team = "Red"
        Turn = "Left"

        error,White_line_detection = line_follower(frame,Line_mask,White_line_detection)
        cmd,Bot_State = Cross_section_detection(P1,P2,P3,error)

        if P1 == 0 and P2 == 0 and P3 == 0:
            Zone = "Retry_Zone"
            cmd = rotate(Turn)
            #ser.write(str(cmd).encode())
        elif P1 > 0 and P2 > 0 and P3 == 0:
            Zone = "Retry_Zone"
            error = 0
        elif P1 > 0 and P2 == 0 and P3 == 0:
            Zone = "Starting_Zone"
            error = 0
        if Bot_State == "Stop":
            cmd = 'S' + ',' + '140,0' + "\r"
            #ser.write(str(cmd).encode())
            print('Stop')
            
    elif np.any(b_mask == 255):
        Team = "Blue"
        Turn = "Right"

        error,White_line_detection = line_follower(frame,Line_mask,White_line_detection)
        cmd,Bot_State = Cross_section_detection(P1,P2,P3,error)

        if P1 == 0 and P2 == 0 and P3 == 0:
            Zone = "Retry_Zone"
            cmd = rotate(Turn)
            #ser.write(str(cmd).encode())
        elif P1 > 0 and P2 == 0 and P3 > 0:
            Zone = "Retry_Zone"
            error = 0
        elif P1 > 0 and P2 == 0 and P3 == 0:
            Zone = "Starting_Zone"
            error = 0
        if Bot_State == "Stop":
            cmd = 'S' + ',' + '140,0' + "\r"
            #ser.write(str(cmd).encode())
            print('Stop')
            
    else:
        Team = 'NA'
        Zone = 'NA'
        error,White_line_detection = line_follower(frame,Line_mask,White_line_detection)
        cmd,Bot_State = Cross_section_detection(P1,P2,P3,error)
        if Bot_State == "Stop":
            cmd = 'S' + ',' + '140,0' + "\r"
            #ser.write(str(cmd).encode())
            print('stop')
            

    #Display the frames
    cv2.imshow('Frame', frame)
    cv2.imshow('Red mask',r_mask)
    cv2.imshow('White mask',m)
    #cv2.imshow("P1 mask",P1_mask)
    #cv2.imshow("P2 mask",P2_mask)
    #cv2.imshow("P3 mask",P3_mask)


    # Check for key press
    key = cv2.waitKey(25)
    if key == 27:  # Press 'Esc' to exit
        break
    
    # Generating an command to send to arduino
    cmd = str(error) + ',' + '140,0' + "\r"
    #ser.write(str(cmd).encode())
    #print(cmd)

    # Print helps what's going on
    print("Team: ",Team,'||',
          "Turn: ",Turn,'||',
          "Zone: ",Zone,'||',
          "White_Line_Detection: ",White_line_detection,'||',
          "  ",
          "P2: ",P2,'||',
          "P1: ",P1,"||",
          "P3: ",P3,"||",
          "Error: ",error,"||",
          "Cmd: ",cmd)

# Release video capture object and close all windows
cap.release()
cv2.destroyAllWindows()
