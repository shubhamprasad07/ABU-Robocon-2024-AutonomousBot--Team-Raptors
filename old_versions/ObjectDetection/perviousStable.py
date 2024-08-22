import numpy as np
import cv2 
from ultralytics import YOLO
from time import time

# Initialize camera
cam = cv2.VideoCapture(0)

loop_time = time()
fps_list = []

# Load YOLO model
model = YOLO('M_BL24_best.pt').to('cuda')

# Dictionary to store counts for each class
class_counts = {}

while True:
    # Read frame from camera
    ret, frame = cam.read()
    if not ret:
        break

    # Create a copy of the frame for annotations
    image = frame.copy()

    # Perform object detection
    results = model(frame, conf=0.6, show=False, verbose=False)

    # Process each detection result
    for result in results:
        class_counts = {}
        max_areas = {}
        max_y_centers = {}
        max_x1s, max_x2s, max_y1s, max_y2s = {}, {}, {}, {}  # Initialize variables for the bounding boxes with max area and max y_center for each class

        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            x_center = (x1 + x2) / 2
            y_center = (y1 + y2) / 2

            area = (x2 - x1) * (y2 - y1)

            center_center_x = x_center 

            # Get class name and ID
            class_id = box.cls[0].item()
            class_name = model.names[class_id]

            # Initialize max values for the class if not already initialized
            if class_name not in max_areas:
                max_areas[class_name] = 0
                max_y_centers[class_name] = 0
                max_x1s[class_name], max_x2s[class_name], max_y1s[class_name], max_y2s[class_name] = 0, 0, 0, 0

            # Update class_counts
            if class_name not in class_counts:
                class_counts[class_name] = 1  # Initialize count for new class
            else:
                class_counts[class_name] += 1  # Increment count for existing class

            # Update max_areas, max_y_centers and bounding box coordinates for the class
            if area > max_areas[class_name] and y_center > max_y_centers[class_name]:
                max_y_centers[class_name] = y_center
                max_x1s[class_name], max_x2s[class_name], max_y1s[class_name], max_y2s[class_name] = x1, x2, y1, y2

            # Print and annotate object details
            print(f"Name-> {class_name} [{class_counts[class_name]}] || Loc-> {x1}, {y1}, {x2}, {y2}")

            # Draw solid box for label background
            label_size, base_line = cv2.getTextSize(f"{class_name} [{class_counts[class_name]}]", cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
            cv2.rectangle(image, (x1, y1 - label_size[1] - 10), (x1 + label_size[0] + 10, y1), (0, 0, 255), -1)

            if class_name == 'cell phone':
                color = (0,0,0)
            else :
                color = (0,0,255)

            # Draw text and bounding box
            cv2.putText(image, f"{class_name} [{class_counts[class_name]}]", (x1 + 5, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

        # Draw bounding boxes with max area and max y_center for each class
        for class_name in max_areas:
            cv2.rectangle(image, (max_x1s[class_name], max_y1s[class_name]), (max_x2s[class_name], max_y2s[class_name]), (0, 255, 0), 2)


    # Calculate and display FPS
    fps = 1 / (time() - loop_time)
    fps_list.append(fps)
    avg_fps = np.mean(fps_list[-20:])  # Calculate average FPS over last 20 frames
    cv2.putText(image, f'FPS: {avg_fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    # Display annotated frame
    cv2.imshow("frame", image)

    # Print and update FPS
    print(f"FPS: {avg_fps:.2f}")
    loop_time = time()

    # Check for 'q' key press to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release camera and close windows
cam.release()
cv2.destroyAllWindows()
