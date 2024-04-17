import cv2
import numpy as np
import serial
import time

# Initialize serial connection
ser = serial.Serial('COM10', 9600,timeout=0.1)  # Change 'COM6' to the correct port and 9600 to the correct baud rate
time.sleep(1)

def main():
    # Open the default camera
    cap = cv2.VideoCapture(1)  # 0 represents the default camera, change it if you have multiple cameras
    
    # Initialize coordinates
    x_coordinate, y_coordinate = 0, 0

    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            break

        # Flip the frame horizontally to create a mirror effect
        frame = cv2.flip(frame, 1)

        # Resize the frame to fit your screen
        frame = cv2.resize(frame, (320, 240))  # Adjust width and height as needed

        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range of blue color in HSV
        blue_min = (60, 50, 66)
        blue_max = (116, 240, 255)

        # Convert the blue_min and blue_max tuples to numpy arrays
        lower_blue = np.array(blue_min)
        upper_blue = np.array(blue_max)

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Apply morphological operations (erosion and dilation) to remove noise
        mask = cv2.erode(mask, None, iterations=7)
        mask = cv2.dilate(mask, None, iterations=3)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize center coordinates
        center_x, center_y = 0, 0

        # Initialize variables to track the ball with the least y-coordinate
        min_y_coordinate = float('inf')  # Set to positive infinity initially
        min_x_coordinate = 0
        min_ball_area = 0

        # Check if any ball is detected
        ball_detected = False

        # Iterate through contours to find circular ones
        for contour in contours:
            # Fit a circle to the contour
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center_x, center_y = int(x), int(y)

            # Calculate area of the contour
            area = cv2.contourArea(contour)

            # Store the coordinates
            x_coordinate = center_x - frame.shape[1] // 2
            y_coordinate = frame.shape[0] // 2 - center_y  # Invert y-axis to keep positive up

            # Update min_y_coordinate if the current ball has a lower y-coordinate
            if y_coordinate < min_y_coordinate:
                min_y_coordinate = y_coordinate
                min_x_coordinate = x_coordinate
                # Map the area of the ball from 0 to 100
                min_ball_area = int(np.clip((area / (frame.shape[0] * frame.shape[1])) * 100, 0, 100))

            # Draw the circle if the radius is within a certain range
            if 10 < radius < 110:
                ball_detected = True

                # Draw the circle and dot at the center
                cv2.circle(frame, (center_x, center_y), int(radius), (255, 0, 0), 2)  # Blue color
                cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)  # Green color

        # Display adjusted coordinates if a blue ball is detected
        if ball_detected:
            # Display coordinates and ball area of the lowest ball
            cv2.putText(frame, f'({min_x_coordinate}, {min_y_coordinate})', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv2.putText(frame, f'Ball Area: {min_ball_area}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            # Send coordinates and area to Arduino
            send_to_arduino(min_x_coordinate, min_y_coordinate, min_ball_area)
        else:
            # No ball detected, send (0, 0, 0) to Arduino
            send_to_arduino(0, 0, 0)

        # Display the frame
        cv2.imshow('Frame', frame)
        cv2.imshow("mask", mask)

        # Read data from Arduino and print on terminal
        if ser.in_waiting > 0:
            arduino_data = ser.readline().decode().strip()
            print("Received data from Arduino:", arduino_data)

        # Add a small delay between frames
        time.sleep(0.02)

        # Press 'q' to exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

    # Close the serial connection and release resources
    ser.close()
    cap.release()
    cv2.destroyAllWindows()

def send_to_arduino(x, y, area):
    # Send coordinates and area to Arduino
    cmd = f'{x},{y},{area}\r'
    print(cmd)
    ser.write(cmd.encode())

if __name__ == "__main__":
    main()