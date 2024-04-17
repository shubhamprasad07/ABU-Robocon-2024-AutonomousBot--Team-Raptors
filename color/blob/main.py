file_path = "/home/raptors/Desktop/robocon/blob/main.py"

import cv2
import time
from blob_detector import *
from config import *
import serial

K_LAT_DIST_TO_STEER = 2.0
K_LAT_TROTTLE = 1.5
K_LAT_ANGLE = 2.0

def saturate(value, min, max):
    if value <= min:
        return min
    elif value >= max:
        return max
    else:
        return value


class BlobDetector:

    def __init__(self, thr_min, thr_max, blur=15, blob_params=None, detection_window=None):
        self.COUNT = 0
        self.set_threshold(thr_min, thr_max)
        self.set_blur(blur)
        self.set_blob_params(blob_params)
        self.detection_window = detection_window
        self._t0 = time.time()
        self.serial_connection = serial.Serial("com10", BAUD_RATE, timeout=1)
        self.min_size = min_size
        self.max_size = max_size
        self.min_throttle = 0.02
        self.max_throttle = 0.98
        self.s = 0.0

    def set_threshold(self, thr_min, thr_max):
        self._threshold = [thr_min, thr_max]

    def set_blur(self, blur):
        self._blur = blur

    def set_blob_params(self, blob_params):
        self._blob_params = blob_params

    def gripper(self ,steer_action, servo_angle, count , s):
        self.COUNT = count
        if (self.COUNT == 0):
            if (-0.1 <= steer_action <= 0.1) and (-0.1 <= servo_angle <= 0.1) and 90 < s < 100:
                gripper_message = "C"
                self.serial_connection.write(gripper_message.encode())
                self.COUNT = 1
                print(" |||| ")
                print("UP")
                print("S:%.2f " % steer_action," A:%.2f " % servo_angle," Detect: ", self.is_detected," C:%d " % self.COUNT)
                print(" |||| ")
                return self.COUNT
            else:
                if self.is_detected:
                    # print(" |||| ")
                    # print("Not in range")
                    # print("S:%.2f " % steer_action,"S:%.2f " % servo_angle,"Detect: ", self.is_detected," S:%d " % self.COUNT) 
                    # print(" |||| ")
                    return self.COUNT
                else:
                    # print(" |||| ")
                    # print("No Detection")
                    # print("S:%.2f " % steer_action,"S:%.2f " % servo_angle,"Detect: ", self.is_detected," S:%d " % self.COUNT)
                    # print(" |||| ")
                    return self.COUNT
        else:
            # print(" |||| ")
            # print("FAIL")
            # print("S:%.2f " % steer_action,"S:%.2f " % servo_angle,"Detect: ", self.is_detected," S:%d " % self.COUNT)
            # print(" |||| ")
            return self.COUNT

    def send_control_actions(self, steer_action, throttle_action, servo_angle , s):

        bb_max = 1.5 * 0.3
        bb_min = -1 * bb_max

        if bb_min <= steer_action <= bb_max:
            steer_action = 0.0
            
        if( s == 0 ):
            throttle_action = 0.0

        data_array = [steer_action*100,throttle_action*100,servo_angle*100]
        message = ','.join(map(str, data_array)) + '\n'
        print("Sended to Arduino :" + message)
        self.serial_connection.write(message.encode())

        try:
            reply = self.serial_connection.readline().decode()
            print("Arduino received:", reply)
        except serial.SerialTimeoutException:
            print("No reply")


    def get_control_action(self, x, y, s, frame_height):
        
        steer_action = 0.0
        throttle_action = 0.0
        servo_angle = 0.0

        if self.is_detected and self.min_size <= s <= self.max_size: 
            steer_action = K_LAT_DIST_TO_STEER * x
            steer_action = saturate(steer_action, -1.5, 1.5)
            steer_action = round(steer_action, 1)
            # print("Steering command %.2f" % steer_action)

            normalized_size= (s - self.min_size) / (self.max_size - self.min_size)
            inverted_size = 1 - normalized_size
            throttle_action = inverted_size * (self.max_throttle - self.min_throttle) + self.min_throttle
            throttle_action = saturate(throttle_action, 0, 1)
            throttle_action = round(throttle_action, 1)
            # throttle_action = round(throttle_action, 1) 
            # print("Throttle action %.2f" % throttle_action)

            
            # Map normalized y-coordinate to servo angle range
            servo_angle = y * K_LAT_ANGLE  # Adjust this factor based on the servo angle range
            # Saturate servo angle to ensure it's within the desired range
            servo_angle = saturate(servo_angle, -1.5, 1.5)
            servo_angle = round(servo_angle, 1)
            # print("Servo Angle %.2f" % servo_angle)

        return steer_action, throttle_action , servo_angle

    def detect_blob(self, cv_image):
        (rows, cols, channels) = cv_image.shape
        x, y, s = 0.0, 0.0 , 0.0
        if cv_image is not None:
            # Detect blobs
            keypoints, mask = blob_detect(cv_image, self._threshold[0], self._threshold[1], self._blur,
                                          blob_params=self._blob_params, search_window=self.detection_window)
            # Draw search window and blobs
            # cv_image = blur_outside(cv_image, 10, self.detection_window)        
            cv_image = draw_window(cv_image, self.detection_window, line=1)
            cv_image = draw_frame(cv_image)
            cv_image = draw_keypoints(cv_image, keypoints)
            ball_count = 0
            
            max_x = -1.50
            max_s = 0
            max_y = -1.50

            for i, keyPoint in enumerate(keypoints):
                # Here you can implement some tracking algorithm to filter multiple detections
                # We are simply getting the first result
                x = keyPoint.pt[0]
                y = keyPoint.pt[1]
                s = keyPoint.size
                
                # Find x and y position in camera adimensional frame
                x, y = get_blob_relative_position(cv_image, keyPoint)
                ball_count += 1

                # Check if the current keypoint has a higher y-coordinate than the maximum found so far
                if y > max_y and self.min_size <= s <= self.max_size:
                    # Update the maximum y-coordinate and its corresponding coordinates
                    max_y = y
                    max_x = x
                    max_s = s
                else:
                    max_x = 0
                    max_y = 0
                    max_s = 0

                    # print("i(%d) :: " % i,"X : %.1f" % max_x," Y : %.1f" % max_y," S : %.1f " % max_s)


            fps = 1.0 / (time.time() - self._t0)
            self._t0 = time.time()
            # return cv_image , mask, x, y, s, fps, ball_count
            return cv_image , mask, max_x, max_y, max_s, fps , ball_count
        
    
    
    @property
    def is_detected(self):
        # print(time.time() - self._t0)
        return time.time() - self._t0 < 1.0
    
    

def main():

    frame_height = WIDTH
    frame_width = WIDTH

    blue_min = (47, 21, 66)
    blue_max = (116, 240, 255)
    # blue_min = (82,31,62)
    # blue_max = (106, 116, 193)
    # blue_min = (55,40,0)
    # blue_max = (150, 255, 255)

    blur = 0

    # detection window respect to camera frame in [x_min, y_min, x_max, y_max] adimensional (0 to 1)

    flag = 0

    detection_window = DETECTION_WINDOW

    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 0
    params.maxThreshold = 100

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 1000
    params.maxArea = 45000

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.05

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.2

    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.7

   

    detector = BlobDetector(blue_min, blue_max, blur, params, detection_window)

    # Replace this with your camera source or image list
    # For example, use cv2.VideoCapture() for a camera stream
    # or load images from a list of file paths

    cap = cv2.VideoCapture(0)  # Open camera
    cap.set(3, frame_width)  # Set resolution
    cap.set(4, frame_height) # Set resolution
    cap.set(5, 30)  # Set frame rate
    # cap.set(21, 0)  # Set auto exposure to manual
    # cap.set(10, 0.5)  # Set brightness to 0.5
    # cap.set(15, -7)  # Set exposure to -7
    # cap.set(11, 0.5)  # Set contrast to 0.5
    # cap.set(12, 0.5)  # Set saturation to 0.5
    # cap.set(13, 0.5)  # Set hue to 0.5
    # cap.set(14, 0.5)  # Set gain to 0.5
    # cap.set(17, 0.5)  # Set white balance to 0.5
    # cap.set(19, 0.5)  # Set backlight to 0.5
    # 

    while True:
        ret, frame = cap.read()  # Capture frame
        frame = cv2.flip(frame, 1)

        if not ret:
            print("Error: Camera failed to capture frame")
            break

        # Pass the captured frame to the detector
        result , negative , x , y , s , fps , ball_count = detector.detect_blob(frame)

        steer_action, throttle_action, servo_angle = detector.get_control_action(x , y , s , frame_height)
        # print("Before line Count", flag)

        if (-0.1 <= steer_action <= 0.1) and (-0.1 <= servo_angle <= 0.1) and s > min_size:
            flag = detector.gripper(steer_action, servo_angle, flag, s)

        # print("After line Count", flag)

        detector.send_control_actions(steer_action, throttle_action, servo_angle, s)

        if result is not None:
            cv2.rectangle(result,(int(BB_MIN_WIDTH ),int(BB_MIN_HEIGHT)),(int(BB_MAX_WIDTH),int(BB_MAX_HEIGHT)),BB_COLOR,2)
            cv2.putText(result, "Ball Count: %d" % ball_count, (5 , 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 100 , 200), 2)
            cv2.putText(result, "FPS: %.2f" % fps, (5 , 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 100 , 200), 2)
            cv2.imshow("Negative", negative)
            cv2.imshow("Result", result)

            # Move the "Result" window to a new location (x, y)
            cv2.moveWindow("Result", 0, 50)
            # Move the "Negative" window to a new location (x, y)
            cv2.moveWindow("Negative", 1000,500)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    
    main()