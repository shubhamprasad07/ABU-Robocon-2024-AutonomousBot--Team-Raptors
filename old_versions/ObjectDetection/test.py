import numpy as np
import cv2 
from ultralytics import YOLO
from time import time
import serial

class Detector:
    def __init__(self, model_path='m_bl_2.pt'):
        self.model = YOLO(model_path).to('cuda')
        self.fps_list = []
        self.loop_time = time()
        self.serial_connection = serial.Serial("com10", 9600 , timeout = 0.2)
        self.className = ""

    def is_detected(self , area):
        if  1 < area < 99:
            return True
        else : 
            return False
    
    def send_arduino(self , message):
        self.serial_connection.write(message.encode())
        print(message)
    
    def get_arduino(self):
        if self.serial_connection.in_waiting:
            reply = self.serial_connection.read(self.serial_connection.in_waiting).decode()
            if reply == '1':
                self.className = "silo"
            else:
                self.className = "blue_ball"
            print(f"RECEIVED: {reply}, className: {self.className}")
            return reply
        else:
            return None

    def arduino_calculation(self , x_center , y_center , max_area , width , height):
        x_center = x_center - (width // 2)
        y_center = (height // 2) - y_center
        max_area = round(max_area) * 100
        int(max_area)
        int(x_center)
        int(y_center)
        gripper_command = 0

        if self.is_detected(max_area):
            message = f'{x_center},{y_center},{max_area},{gripper_command}\r'
        else:
            message = f'{0},{0},{0},{0}\r'

        # message = ','.join(map(str,data_array)) + '\r'
        return message

    def detect_objects(self, frame):
        results = self.model(frame, conf=0.6, show=False, verbose=False)
        image = frame.copy()
        class_counts = {}
        # Initialize dictionaries to track properties of bounding boxes for each class
        max_areas = {}
        max_x_centers = {}
        max_y_centers = {}
        max_x1s, max_x2s, max_y1s, max_y2s = {}, {}, {}, {}
        x_center, y_center , max_area = 0 , 0 , 0
        x1, y1, x2, y2 = 0 , 0 , 0, 0
        frame_area = frame.shape[1] * frame.shape[0] 
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                x_center, y_center = (x1 + x2) / 2, (y1 + y2) / 2
                area = (x2 - x1) * (y2 - y1)
                class_id = box.cls[0].item()
                class_name = self.model.names[class_id]
                print(class_name)
                if class_name not in class_counts:
                    class_counts[class_name] = 1  # Initialize count for new class
                else:
                    class_counts[class_name] += 1  # Increment count for existing class

                if class_name not in max_areas:
                    max_areas[class_name] = 0
                    max_x_centers[class_name] = 0
                    max_y_centers[class_name] = 0
                    max_x1s[class_name], max_x2s[class_name], max_y1s[class_name], max_y2s[class_name] = 0, 0, 0, 0

                if area > max_areas[class_name] and y_center > max_y_centers[class_name]:
                    max_areas[class_name] = area
                    max_x_centers[class_name] = x_center
                    max_y_centers[class_name] = y_center
                    max_x1s[class_name], max_x2s[class_name], max_y1s[class_name], max_y2s[class_name] = x1, x2, y1, y2

                # Annotations
                color = (0,0,0) if class_name == 'silo' else (0,0,255)
                # cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

        for class_name in max_areas:
            label_size, _ = cv2.getTextSize(f"{class_name} [{class_counts[class_name]}]", cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
            cv2.rectangle(image, (x1, y1 - label_size[1] - 10), (x1 + label_size[0] + 10, y1), (0, 0, 255), -1)
            cv2.putText(image, f"{class_name} [{class_counts[class_name]}]", (x1 + 5, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
            cv2.rectangle(image, (max_x1s[class_name], max_y1s[class_name]), (max_x2s[class_name], max_y2s[class_name]), color, 2)
            x_center = max_x_centers[class_name] 
            y_center = max_y_centers[class_name] 
            max_area = round(max_areas[class_name] / frame_area)
            # print( f"X : {x_center} , Y : {y_center} , S : {max_area}" )
        return image , x_center , y_center , max_area




    def run(self):
        cam = cv2.VideoCapture(1)
        while True:
            ret, frame = cam.read()
            if not ret:
                break
            
            reply = self.get_arduino()
            # if reply is not None:
            #     print(f"RECIVED: {reply}")

            image , x_center , y_center , max_area = self.detect_objects(frame)

            message = self.arduino_calculation(x_center , y_center , max_area , frame.shape[1] , frame.shape[0])

            self.send_arduino(message)

            # time.sleep(0.1)

            fps = 1 / (time() - self.loop_time)
            self.fps_list.append(fps)
            avg_fps = np.mean(self.fps_list[-20:])
            cv2.putText(image, f'FPS: {avg_fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            cv2.imshow("frame", image)

            self.loop_time = time()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cam.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    detector = Detector()
    detector.run()
