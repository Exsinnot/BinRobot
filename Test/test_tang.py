import cv2
import mediapipe as mp

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

# Start video capture
cap = cv2.VideoCapture(0)

def is_thumb_up(hand_landmarks):
    # Get landmark positions for the thumb and other fingers
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    thumb_ip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP]
    
    # Get positions for other fingers' tips and their respective bases
    index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    index_dip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP]
    
    middle_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    middle_dip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP]
    
    ring_tip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
    ring_dip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_DIP]
    
    pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
    pinky_dip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_DIP]
    
    # Check if thumb is up (thumb_tip should be higher than thumb_ip)
    thumb_up = thumb_tip.y < thumb_ip.y
    fuck = middle_tip.y > middle_dip.y
    
    # Check if other fingers are down (tip of the finger should be below its base)
    index_down = index_tip.y > index_dip.y
    middle_down = middle_tip.y > middle_dip.y
    ring_down = ring_tip.y > ring_dip.y
    pinky_down = pinky_tip.y > pinky_dip.y
    
    return thumb_up and index_down and middle_down and ring_down and pinky_down

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # Convert the image to RGB for MediaPipe
    image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process the image and detect hand landmarks
    results = hands.process(image_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Draw hand landmarks on the image
            mp.solutions.drawing_utils.draw_landmarks(
                frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            # Check if the hand is showing a thumbs-up gesture
            if is_thumb_up(hand_landmarks):
                # cv2.putText(frame, "Thumbs Up Detected!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                print('Thump')
            else:
                print('dddd')
    
    # Display the image
    # cv2.imshow('Thumbs Up Detection', frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
