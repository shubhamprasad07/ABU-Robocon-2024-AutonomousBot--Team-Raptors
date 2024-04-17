import numpy as np
import cv2 
from ultralytics import YOLO
from time import time
import serial

com = "com10"

class Detector:
    def __init__(self, model_path='m_bl_2.pt'):
        self.model = YOLO(model_path).to('cuda')
        self.fps_list = []
        self.loop_time = time()
        self.serial_connection = serial.Serial(com, 9600 , timeout = 0.2)
        self.className = "0"

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
            # reply = self.serial_connection.read(self.serial_connection.in_waiting).decode()
            reply = self.serial_connection.read(self.serial_connection.in_waiting)
            if reply == '1':
                self.className = "1"
            else:
                self.className = "0"
            print(f"RECEIVED: {reply}, className: {self.className}")
            return reply
        else:
            return None

    def arduino_calculation(self , x_center , y_center , max_area , width , height):
        x_center = x_center - (width // 2)
        y_center = (height // 2) - y_center
        max_area = round(max_area / (width * height) , 2 ) * 100
        # print("max_area : ",max_area)
        int(max_area)
        int(x_center)
        int(y_center)
        gripper_command = 0

        if self.is_detected(max_area):
            message = f'{x_center},{y_center},{max_area},{gripper_command}\r'
            # print("if : " , message )
        else:
            message = f'{0},{0},{0},{0}\r'

        # message = ','.join(map(str,data_array)) + '\r'
        return message

    def detect_objects(self, frame):
        results = self.model(frame, conf=0.6, show=False, verbose=False)
        image = frame.copy()
        class_counts = {}
        max_areas = 0
        max_x_centers = -400
        max_y_centers = -300
        max_x1s, max_x2s, max_y1s, max_y2s = 0,0,0,0
        x_center, y_center , max_area = 0,0,0
        x1, y1, x2, y2 = 0 , 0 , 0, 0
        frame_area = frame.shape[1] * frame.shape[0] 

        # Inside the detect_objects method
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                x_center, y_center = (x1 + x2) / 2, (y1 + y2) / 2
                # print(x_center, y_center)
                area = (x2 - x1) * (y2 - y1)
                class_id = box.cls[0].item()
                class_name = self.model.names[class_id]
                # print(self.className)
                if self.className == "1":
                    if area > max_areas :
                        max_areas = area
                        max_x_centers = x_center
                        max_y_centers = y_center
                        max_x1s, max_x2s, max_y1s, max_y2s = x1, x2, y1, y2
                        # cv2.circle(image, (x_center, y_center), 2, (0, 0, 0), 2)
                        cv2.rectangle(image, (max_x1s, max_y1s), (max_x2s, max_y2s), (0, 0, 255), 3)
                        return image , max_x_centers , max_y_centers , max_areas

                elif self.className == "0":
                    if area > max_areas :
                        max_areas = area
                        max_x_centers = x_center
                        max_y_centers = y_center
                        # cv2.circle(image, (x_center, y_center), 2, (0, 0, 0), 2)
                        max_x1s, max_x2s, max_y1s, max_y2s = x1, x2, y1, y2
                        cv2.rectangle(image, (max_x1s, max_y1s), (max_x2s, max_y2s), (0, 0, 255), 3)
                        return image , max_x_centers , max_y_centers , max_areas
        return image, 0, 0, 0

        #         # Update class counts
        #         class_counts[class_name] = class_counts.get(class_name, 0) + 1

        # # Draw labels based on class_counts
        # # Drawing Labels
        # for class_name, count in class_counts.items():
        #     label_size, _ = cv2.getTextSize(f"{class_name} [{count}]", cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        #     cv2.rectangle(image, (max_x1s, max_y1s - label_size[1] - 10), 
        #                   (max_x1s + label_size[0] + 10, max_y1s), (0, 0, 255), -1)
        #     cv2.putText(image, f"{class_name} [{count}]", (max_x1s + 5, max_y1s - label_size[1] // 2 - 5), 
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
        # # Drawing Bounding Box
        # cv2.rectangle(image, (max_x1s, max_y1s), (max_x2s, max_y2s), (0, 0, 255), 3)
        # return image , max_x_centers , max_y_centers , max_areas , class_name


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
            # print("run : " , x_center , y_center , max_area)

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
