import cv2 as cv
from Def import VarGlobal
import time
import numpy as np

net = cv.dnn.readNet('yolov4-tiny.weights', 'yolov4-tiny.cfg')
with open('coco.names', 'r') as f:
    classes = f.read().strip().split('\n')
    
def Detect():
    global net
    if VarGlobal.frame is None:
        return None,None,None,None
    height, width = VarGlobal.frame.shape[:2]
    blob = cv.dnn.blobFromImage(VarGlobal.frame, 1/255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
    detections = net.forward(output_layers)

    boxes, confidences, class_ids = [], [], []
    for out in detections:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                box = detection[0:4] * np.array([width, height, width, height])
                (centerX, centerY, w, h) = box.astype("int")
                x = int(centerX - (w / 2))
                y = int(centerY - (h / 2))
                boxes.append([x, y, int(w), int(h)])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    indices = cv.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    return indices,class_ids,boxes,confidences

def Follow_Mode():
    global classes
    while True:
        indices,class_ids,boxes,confidences = Detect()
        try:
            if len(indices) > 0:
                for i in indices.flatten():
                    x, y, w, h = boxes[i]
                    label = str(classes[class_ids[i]])
                    confidence = confidences[i]
                    color = (0, 255, 0)
                    cv.rectangle(VarGlobal.frameweb, (x, y), (x + w, y + h), color, 2)
                    if label == "person":
                        print("person")
        except:
            break