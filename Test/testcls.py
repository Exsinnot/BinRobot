import cv2
import mediapipe as mp
import numpy as np
import time
mode = True
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
def detect_hand():
    global mode
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_alt2.xml')
    capture = cv2.VideoCapture(0)

    with mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as hands:
        while capture.isOpened():
            settime = time.time()
            if mode:
                success, image = capture.read()
                image = cv2.resize(image, (1280, 960))
                image = cv2.flip(image, 1)
                gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                brightness = cv2.mean(gray_image)[0]
                if brightness < 120:
                    alpha = 120/brightness
                    image = np.clip(image * alpha, 0, 255).astype(np.uint8)
                elif brightness > 130:
                    alpha = 130/brightness
                    image = np.clip(image * alpha, 0, 255).astype(np.uint8)
                gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                brightness = cv2.mean(gray_image)[0]
                # print(brightness)
                height, width, _ = image.shape

                image.flags.writeable = False  
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = hands.process(image)
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                if results.multi_hand_landmarks:
                    hand_landmarks = results.multi_hand_landmarks[0]
                    landmark = hand_landmarks.landmark[0]
                    height, width, _ = image.shape
                    cx, cy = int(landmark.x * width), int(landmark.y * height)
                    # print(f'Landmark {0}: ({cx}, {cy})')
                    if cx < (width/2) + (width//10) and cx > (width/2) - (width//10):
                        textcom = " OK "
                    elif cx < (width/2):
                        textcom = " Right "
                    elif cx > (width/2):
                        textcom = " Left "
                    if cy < (height/2) + (height//10) and cy > (height/2) - (height//10):
                        textcom += "OK "
                    elif cy < (height/2):
                        textcom += "Down "
                    elif cy > (height/2):
                        textcom += "Up "
                    cv2.rectangle(image, (int(width/2)-(height//10), int(height/2)-(height//10)), (int(width/2)+(width//10), int(height/2)+(width//10)), (0, 255, 0), 2)
                    cv2.putText(image, str(cx)+","+str(cy)+textcom, (50, 450), cv2.FONT_HERSHEY_COMPLEX_SMALL, 3, (255, 0, 0), 5)
                    print("เจอมือแบบไกล้ๆ")
                else:  
                    faces = face_cascade.detectMultiScale(gray_image, scaleFactor=1.1, minNeighbors=5)
                    for (x, y, w, h) in faces:
                        # print(x,y,w,h)
                        if y - h*4 < 0:
                            y = 0
                        else:
                            y = y - h*4
                        if h*5 > height:
                            h = height
                        else:
                            h = h*5
                        
                        if x - w*4 < 0:
                            x = 0
                        else:
                            x = x - w*4
                        if w*8 > width:
                            w = width
                        else:
                            w = w*8
                        submit = image[y:y+h, x:x+w]
                        cv2.rectangle(image, (x, y), (x+w,y+h), (0, 255, 0), 3)
                        submit.flags.writeable = False
                        submit = cv2.cvtColor(submit, cv2.COLOR_BGR2RGB)
                        results = hands.process(submit)
                        submit.flags.writeable = True
                        submit = cv2.cvtColor(submit, cv2.COLOR_BGR2RGB)
                        
                        if results.multi_hand_landmarks:
                            hand_landmarks = results.multi_hand_landmarks[0]
                            landmark = hand_landmarks.landmark[0]
                            height2, width2, _ = submit.shape
                            height, width, _ = image.shape
                            cx, cy = int(landmark.x * width2)+x, int(landmark.y * height2)+y
                            # print(f'Landmark {0}: ({cx}, {cy})')
                            if cx < (width/2) + (width//10) and cx > (width/2) - (width//10):
                                textcom = " OK "
                            elif cx < (width/2):
                                textcom = " Right "
                            elif cx > (width/2):
                                textcom = " Left "
                            if cy < (height/2) + (height//10) and cy > (height/2) - (height//10):
                                textcom += "OK "
                            elif cy < (height/2):
                                textcom += "Down "
                            elif cy > (height/2):
                                textcom += "Up "
                            cv2.rectangle(image, (int(width/2)-(height//10), int(height/2)-(height//10)), (int(width/2)+(width//10), int(height/2)+(width//10)), (0, 255, 0), 2)
                            cv2.putText(image, str(cx)+","+str(cy)+textcom, (50, 450), cv2.FONT_HERSHEY_COMPLEX_SMALL, 3, (255, 0, 0), 5)
                            print("เจอมือแบบไกลๆ")
                            break
                print(time.time() - settime)
                # cv2.imshow('P', image)
                # if cv2.waitKey(1) == ord('q') or not mode:
                #     cv2.destroyAllWindows()
        capture.release()
detect_hand()