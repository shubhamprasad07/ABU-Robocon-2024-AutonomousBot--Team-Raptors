import numpy as np
import cv2 
from ultralytics import YOLO
from time import time
import serial

com = "com9"

class Detector:
    def __init__(self, model_path='m_bl_2.pt'):
        self.model = YOLO(model_path).to('cuda')
        self.fps_list = []
        try:
            self.serial_connection = serial.Serial(com, 9600, timeout=0.1)
        except serial.SerialException as e:
            print(f"Failed to connect to {com}: {e}")
            raise
        self.className = "blue_ball"
        self.count = 1

    def is_detected(self , area): 
        return  1 <= area <= 100
            
    
    def send_arduino(self , message):
        try:
            self.serial_connection.write(message.encode())
            # print(f"Sent: {message}")
        except serial.SerialException:
            print("Failed to send message to Arduino.")
    
    def get_arduino(self):
        if self.serial_connection.in_waiting:
            reply_bytes = self.serial_connection.read(self.serial_connection.in_waiting)
            try:
                reply = reply_bytes.decode().strip()
                if reply == '1':
                    self.className = "silo"
                else:
                    if self.count % 2 == 0:
                        self.className = "purple_ball"
                    else:
                        self.className = "blue_ball"
                    self.count = self.count + 1
                print(f"RECEIVED: {reply} , Counter : {self.count} , ClassName: {self.className}")
                return reply
            except UnicodeDecodeError:
                print("Failed to decode received data.")
                return None
        else:
            return None

    def arduino_calculation(self , x_center , y_center , max_area , width , height):
        x_center = x_center - (width // 2)
        y_center = (height // 2) - y_center
        # print(f"MaxArea : {max_area}")
        max_area = round(max_area / (width * height) , 2 ) * 100
        # print(f"MaxArea 2 : {max_area}")
        
        max_area = int(max_area)
        x_center = int(x_center)
        y_center = int(y_center)

        if self.is_detected(max_area):
            message = f'{x_center},{y_center},{max_area}\r'
            # print("if : " , message )
        else:
            message = f'{0},{0},{0}\r'

        # message = ','.join(map(str,data_array)) + '\r'
        return message

    def detect_objects(self, frame):
        results = self.model(frame, conf=0.8, show=False, verbose=False)
        image = frame.copy()
        class_counts = {}

        # Initialize dictionaries to track properties of bounding boxes for each class
        max_areas = {}
        max_x_centers = {}
        max_y_centers = {}
        max_count = {}
        max_x1s, max_x2s, max_y1s, max_y2s = {}, {}, {}, {}

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                x_center, y_center = (x1 + x2) / 2, (y1 + y2) / 2
                area = (x2 - x1) * (y2 - y1)
                class_id = box.cls[0].item()
                class_name = self.model.names[class_id]
                

                if class_name not in class_counts:
                    class_counts[class_name] = 1  # Initialize count for new class
                else:
                    class_counts[class_name] += 1  # Increment count for existing class

                if class_name not in max_areas:
                    max_areas[class_name] = 0
                    max_x_centers[class_name] = 0
                    max_y_centers[class_name] = 0
                    max_count[class_name] = 1
                    max_x1s[class_name], max_x2s[class_name], max_y1s[class_name], max_y2s[class_name] = 0, 0, 0, 0

                if area > max_areas[class_name] and y_center > max_y_centers[class_name]:
                    max_x_centers[class_name] = x_center
                    max_y_centers[class_name] = y_center
                    max_x1s[class_name], max_x2s[class_name], max_y1s[class_name], max_y2s[class_name] = x1, x2, y1, y2
                    max_areas[class_name] = area
                
                color1 = ( 0 , 255 , 0 ) #Green max with mode
                color2 = ( 0 , 0 , 255 ) #Red mode
                color3 = ( 255 , 0 , 0 ) #Blue not mode

                # Annotations
                if (class_name == self.className):
                    label_size, _ = cv2.getTextSize(f"{class_name} [{class_counts[class_name]}]", cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
                    cv2.rectangle(image, (x1, y1 - label_size[1] - 10), (x1 + label_size[0] + 10, y1), color2, -1)
                    cv2.putText(image, f"{class_name} [{class_counts[class_name]}]", (x1 + 5, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
                    cv2.rectangle(image, (x1, y1), (x2, y2), color2, 2)

                if (class_name != self.className):
                    label_size, _ = cv2.getTextSize(f"{class_name} [{class_counts[class_name]}]", cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
                    cv2.rectangle(image, (x1, y1 - label_size[1] - 10), (x1 + label_size[0] + 10, y1), color3, -1)
                    cv2.putText(image, f"{class_name} [{class_counts[class_name]}]", (x1 + 5, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
                    cv2.rectangle(image, (x1, y1), (x2, y2), color3, 2)


        for class_name in max_areas:
            if (class_name == self.className):
                cv2.rectangle(image, (max_x1s[class_name], max_y1s[class_name]), (max_x2s[class_name], max_y2s[class_name]), color1, 4)
                # print(f" X : {max_x_centers[class_name]} , Y : {max_y_centers[class_name]} , A : {max_areas[class_name]}")
                return image , max_x_centers[class_name] , max_y_centers[class_name] , max_areas[class_name]

        return image , 0 , 0 , 0

    def run(self):

        try:
            cam = cv2.VideoCapture(1)
            if not cam.isOpened():
                raise Exception("Error: Could not open camera.")
        except Exception as e:
            print(e)
            return
        
        while True:
            try:
                ret, frame = cam.read()
                if not ret:
                    print("Error: Failed to grab frame.")
                    break
            except Exception as e:
                print(e)
                break

            start_time = time()

            reply = self.get_arduino()

            image , x_center , y_center , max_area = self.detect_objects(frame)

            message = self.arduino_calculation(x_center , y_center , max_area , frame.shape[1] , frame.shape[0])

            self.send_arduino(message)

            end_time = time()
            fps = 1 / (end_time - start_time)
            self.fps_list.append(fps)
            avg_fps = np.mean(self.fps_list[-20:])
            cv2.putText(image, f'FPS: {avg_fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            cv2.imshow("frame", image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
 
        try:
            cam.release()
            cv2.destroyAllWindows()
        except Exception as e:
            print(e)