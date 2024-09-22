import cv2
from ultralytics import YOLO

# Load YOLOv8 model (use 'yolov8n.pt' for a small, pre-trained model or your own trained model)
model = YOLO('/home/user/BinRobot/Test/best.pt')  # or your model path

# Open a video capture (webcam in this case)
cap = cv2.VideoCapture(0)

while True:
    # Read frame from webcam
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        break

    # Perform inference
    results = model(frame)

    # Annotate frame with detection results
    # annotated_frame = results[0].plot()  # Automatically plots detection boxes on the frame

    # Display the output frame
    #cv2.imshow("YOLOv8 Detection", annotated_frame)

    # Exit on 'q' key press
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

# Release the capture and destroy windows
cap.release()
cv2.destroyAllWindows()
