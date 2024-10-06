import cv2 as cv
from Def import VarGlobal
import time

def read_camera():
    VarGlobal.cap = cv.VideoCapture(0)
    VarGlobal.cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
    VarGlobal.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
    while VarGlobal.cap.isOpened():
        ret, VarGlobal.frame = VarGlobal.cap.read()
        if not ret:
            break
        VarGlobal.frameweb = VarGlobal.frame.copy()
        time.sleep(0.033) 
