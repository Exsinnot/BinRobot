import cv2 as cv
from Def import VarGlobal
import time

def read_camera():
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("EERO")
            break
        VarGlobal.frame = frame.copy()
        VarGlobal.frameweb = frame.copy()
        time.sleep(0.033) 
    print("Can not open camera")
