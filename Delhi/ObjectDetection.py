import numpy as np
import cv2
from ultralytics import YOLO
from time import time
import serial

conf=0.25
desired_width = 1980
desired_height = 1080
com = "com14"
team = "blue"
camera = 1


class Detector:
    def __init__(self, model_path='C:\\Users\\Sanket jain\\OneDrive\\Desktop\\shubham\\AutonomousBot-main\\ObjectDetection\\last(5).pt'):
        self.model = YOLO(model_path).to('cuda')
        self.fps_list = []
        self.mode = team
        self.serial_connection = self.connect_serial(com)

    def connect_serial(self, com):
        try:
            return serial.Serial(com, 9600, timeout=0.1)
        except serial.SerialException as e:
            print(f"Failed to connect to {com}: {e}")
            raise

    def is_detected(self, x , y , area):
        return x > -900 and y < 500
    
    def send_arduino(self, message):
        try:
            self.serial_connection.write(message.encode())
        except serial.SerialException:
            print("Failed to send message to Arduino.")
    
    def get_arduino(self):
        if self.serial_connection.in_waiting:
            reply_bytes = self.serial_connection.read(self.serial_connection.in_waiting)
            try:
                reply = reply_bytes.decode().strip()
                print(type(reply))
                if reply == "1":
                    self.mode = "silo"
                    print(f"If: {reply}e , Mode: {self.mode}")
                else:
                    self.mode = team
                    print(f"else: {reply}e , Mode: {self.mode}")
                # print(f"RECEIVED: {self.mode} , Mode: {self.mode}")
                return reply
            except UnicodeDecodeError:
                print("Failed to decode received data.")
                return None
        return None

    def arduino_calculation(self, x_center, y_center, max_area, width, height):
        x_center = x_center - (width // 2)
        y_center = (height // 2) - y_center
        max_area = int(round(max_area / (width * height), 2) * 100)
        x_center, y_center = int(x_center), int(y_center)
        
        if self.is_detected( x_center, y_center , max_area):
            message = f'{x_center},{y_center},{max_area}\r'
        else:
            message = f'{0},{0},{0}\r'
        # print(f"Send : {message}")
        return message

    def detect_objects(self, frame):
        results = self.model(frame, conf, show=False, verbose=False)
        balls, silos = self.process_results(results)
        if self.mode == "silo":
            frame, x_center, y_center, max_area = self.handle_silo_mode(frame, balls, silos)
        else:
            frame, x_center, y_center, max_area = self.handle_ball_mode(frame, balls)
        return frame, x_center, y_center, max_area

    def process_results(self, results):
        balls, silos = [], []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                label = self.model.names[int(box.cls[0].item())]
                if label in ['blue', 'red']:
                    balls.append((x1, y1, x2, y2, label))
                elif label == 'silo':
                    silos.append((x1, y1, x2, y2, label))
        return balls, silos

    def handle_silo_mode(self, frame, balls, silos):
        silo_ball_count = self.count_balls_in_silos(balls, silos)
        silo_vector, best_silo = self.calculate_silo_scores(silo_ball_count, silos)
        if best_silo != -1:
            return self.draw_silo_info(frame, balls, silos, silo_ball_count, silo_vector, best_silo)
        else:
            x_center, y_center, max_area = 0, 0, 0
            return frame, x_center, y_center, max_area

    def count_balls_in_silos(self, balls, silos):
        silo_ball_count = {i: {'red': 0, 'blue': 0, 'positions': []} for i in range(len(silos))}
        for i, silo in enumerate(silos):
            silo_box = silo[:4]
            for ball in balls:
                center = self.get_center(ball[:4])
                if self.is_inside(center, silo_box):
                    ball_type = ball[4]
                    silo_ball_count[i][ball_type] += 1
                    silo_ball_count[i]['positions'].append((ball_type, center[1]))

            silo_ball_count[i]['positions'].sort(key=lambda x: x[1], reverse=True)
        return silo_ball_count

    def calculate_silo_scores(self, silo_ball_count, silos):
        silo_vector = [[0 for _ in range(len(silos))] for _ in range(4)]
        for i, silo in enumerate(silos):
            for level, (ball_type, _) in enumerate(silo_ball_count[i]['positions']):
                if level < 3:  # Ensure there are no more than 3 levels
                    ball_char = 'R' if ball_type == 'red' else 'B'
                    silo_vector[level + 1][i] = ball_char
                    silo_vector[0][i] += 1
        
        scores = []

        for i, _ in enumerate(silos):
            red_count = sum(1 for row in silo_vector[1:4] if row[i] == 'R')
            blue_count = sum(1 for row in silo_vector[1:4] if row[i] == 'B')
            ball_count = silo_vector[0][i]

            if ball_count >= 3:
                score = float('inf')                # No preference
            elif red_count == 2 and blue_count == 0:
                score = 5
            elif red_count == 0 and blue_count == 2:
                score = 6
            elif red_count == 1 and blue_count == 0:
                score = 1                           # Most preferable
            elif red_count == 1 and blue_count == 1:
                score = 2
            elif red_count == 0 and blue_count == 1:
                score = 4
            elif red_count == 0 and blue_count == 0:
                score = 3
            
            else:
                score = float('inf')               # No preference

            scores.append((score, i))
        if not scores:
            best_silo = -1
        else:
            best_silo = min(scores, key=lambda x: x[0])[1]
        return silo_vector, best_silo

    def draw_silo_info(self, frame, balls, silos, silo_ball_count, silo_vector, best_silo):
        x_center, y_center, area = 0, 0, 0
        scores = []
        for ball in balls:
            x1, y1, x2, y2, label = ball
            center = self.get_center(ball[:4])
            ball_level = 0
            for i, silo in enumerate(silos):
                if self.is_inside(center, silo[:4]):
                    for idx, pos in enumerate(silo_ball_count[i]['positions']):
                        if pos[1] == center[1]:
                            ball_level = idx + 1
                            break
                    break

            color = (0, 0, 255) if label == 'red' else (255, 0, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{label} L{ball_level}" if ball_level > 0 else label, 
                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Calculate scores for silos
        for i, silo in enumerate(silos):
            red_count = sum(1 for row in silo_vector[1:4] if row[i] == 'R')
            blue_count = sum(1 for row in silo_vector[1:4] if row[i] == 'B')
            ball_count = silo_vector[0][i]

            if ball_count >= 3:
                score = float('inf')  # No preference
            elif red_count == 2 and blue_count == 0:
                score = 5
            elif red_count == 0 and blue_count == 2:
                score = 6
            elif red_count == 1 and blue_count == 0:
                score = 1  # Most preferable
            elif red_count == 1 and blue_count == 1:
                score = 2
            elif red_count == 0 and blue_count == 1:
                score = 4
            elif red_count == 0 and blue_count == 0:
                score = 3
            else:
                score = float('inf')  # No preference

            scores.append((score, i))

        best_silo = max(scores, key=lambda x: x[0])[1] if scores else -1

        for i, silo in enumerate(silos):
            sx1, sy1, sx2, sy2 = silo[:4]
            info = f"Silo {i+1} - Red: {silo_ball_count[i]['red']} Blue: {silo_ball_count[i]['blue']}"
            positions_info = " ".join([f"L{idx+1}-{pos[0]}" for idx, pos in enumerate(silo_ball_count[i]['positions'])])
            color = (0, 255, 0) if i == best_silo else (0, 255, 255)
            cv2.rectangle(frame, (sx1, sy1), (sx2, sy2), color, 2)
            cv2.putText(frame, info, (sx1, sy1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.putText(frame, positions_info, (sx1, sy1 - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            if i == best_silo:
                x_center, y_center = self.get_center(silo[:4])
                area = self.get_area(silo[:4])
                
        return frame, x_center, y_center, area

    def handle_ball_mode(self, frame, balls):
        class_counts = {}
        max_info = {}

        for ball in balls:
            x1, y1, x2, y2, label = ball
            area = self.get_area(ball[:4])
            x_center, y_center = self.get_center(ball[:4])
            
            # Increment class count
            if label in class_counts:
                class_counts[label] += 1
            else:
                class_counts[label] = 1

            # Update max_info if this is the largest area seen so far for this label
            if label not in max_info or (area > max_info[label]['area'] and y_center > max_info[label]['y_center']):
                max_info[label] = {'area': area, 'x_center': x_center, 'y_center': y_center, 'box': (x1, y1, x2, y2)}

            self.draw_annotations(frame, ball, class_counts[label])

        # Highlight the largest detected object for the current mode
        if self.mode in max_info:
            x1, y1, x2, y2 = max_info[self.mode]['box']
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 4)
            return frame, max_info[self.mode]['x_center'], max_info[self.mode]['y_center'], max_info[self.mode]['area']
        
        return frame, 0, 0, 0

    def draw_annotations(self, frame, ball, count):
        x1, y1, x2, y2, label = ball
        color = (255, 0, 0) if label == 'blue' else (0, 0, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, f"{label} {count}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    def get_center(self, box):
        x1, y1, x2, y2 = box
        return (x1 + x2) // 2, (y1 + y2) // 2

    def get_area(self, box):
        x1, y1, x2, y2 = box
        return (x2 - x1) * (y2 - y1)

    def is_inside(self, point, box):
        px, py = point
        x1, y1, x2, y2 = box
        return x1 <= px <= x2 and y1 <= py <= y2

    def run(self):
        cap = cv2.VideoCapture(camera)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)

        if not cap.isOpened():
            print("Error: Could not open camera.")
            return

        while True:
            start_time = time()
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            reply = self.get_arduino()
            frame, x_center, y_center, max_area = self.detect_objects(frame)

            message = self.arduino_calculation(x_center, y_center, max_area, frame.shape[1], frame.shape[0])
            self.send_arduino(message)

            self.update_fps(time() - start_time)
            self.display_fps(frame)

            cv2.imshow("Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    def update_fps(self, elapsed_time):
        self.fps_list.append(1 / elapsed_time)
        if len(self.fps_list) > 20:
            self.fps_list.pop(0)

    def display_fps(self, frame):
        if self.fps_list:
            avg_fps = sum(self.fps_list) / len(self.fps_list)
            cv2.putText(frame, f"FPS: {avg_fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

if __name__ == "__main__":
    try:
        detector = Detector()
        detector.run()
    except Exception as e:
        print(f"An error occurred: {e}")
