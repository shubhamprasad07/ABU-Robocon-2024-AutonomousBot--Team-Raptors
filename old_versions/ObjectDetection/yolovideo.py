import cv2
import numpy as np
import torch
import onnxruntime as ort

# Check if CUDA is available and set the device
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# Initialize ONNX runtime session with the ONNX model
ort_session = ort.InferenceSession('best.onnx')

# Open the video capture device
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()  # Read a frame from the video

    if not ret:  # Check if the frame was successfully read
        break

    # Convert the frame to the correct format expected by the ONNX model
    input_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    input_frame = cv2.resize(input_frame, (640, 640))  # Resize to match the model input size
    input_frame = np.transpose(input_frame, (2, 0, 1))  # Transpose to channel-first format
    input_frame = input_frame.astype(np.float32) / 255.0 # Convert to float and normalize
    input_frame = np.expand_dims(input_frame, axis=0)

    # Run inference using the ONNX model
    ort_inputs = {ort_session.get_inputs()[0].name: input_frame}
    outputs = ort_session.run(None, ort_inputs)

    # Process the outputs (e.g., post-processing, drawing bounding boxes)
    # For now, let's assume the outputs are bounding box coordinates and class scores
    # You might need to adjust this part based on the model's output format
    # Here's a simple way to visualize the bounding boxes (you can replace this with your own code)
    
    output_tensor = outputs[0] # Get the single output tensor
    num_detections = output_tensor.shape[1] # Get the number of detected objects
    print(" output_tensor : " , output_tensor)
    print("num :" ,num_detections)
    confidence_threshold = 0.5  # Adjust this value as needed

    for i in range(num_detections):
        box = output_tensor[0, i, :4]  # Extract the bounding box coordinates
        score = output_tensor[0, i, 4]  # Extract the score (or class confidence)
        print("Score : ", score)

        if score > confidence_threshold:
            # Denormalize or adjust the bounding box coordinates if needed
            x1, y1, x2, y2 = box.astype(int)
            # Draw the bounding box and display the score
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"Score: {score:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the processed frame
    cv2.imshow('YOLO', frame)

    key = cv2.waitKey(1)
    if key == ord("q"):
        break

cap.release()  # Release the video capture device
cv2.destroyAllWindows()  # Close all OpenCV windows
