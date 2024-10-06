import cv2
import mediapipe as mp
import time
from pydub import AudioSegment
from io import BytesIO
import pygame

import random
# เรียกใช้โมดูล MediaPipe Pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

# เปิดกล้อง
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    t = time.time_ns()
    results = pose.process(frame_rgb)
    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark

        left_hand_y = landmarks[mp_pose.PoseLandmark.LEFT_WRIST].y
        right_hand_y = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].y
        left_shoulder_y = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y
        right_shoulder_y = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y
        head = landmarks[mp_pose.PoseLandmark.NOSE].y

        if left_hand_y < head or right_hand_y < head:
            sound_list = ['test2.wav',"test.wav","test3.wav"]
            # โหลดและปรับระดับความดังของไฟล์เสียง MP3 ด้วย PyDub
            mp3_audio = AudioSegment.from_mp3(sound_list[random.randint(2,2)])
            mp3_audio = mp3_audio + 10  # เพิ่มความดัง 10 dB

            wav_io = BytesIO()
            mp3_audio.export(wav_io, format="wav")

            wav_io.seek(0)
            pygame.mixer.init()
            pygame.mixer.music.load(wav_io, 'wav')

            pygame.mixer.music.play()

            # รอให้เสียงเล่นเสร็จ
            while pygame.mixer.music.get_busy():
                continue
            break

        mp.solutions.drawing_utils.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
    print((time.time_ns() - t)/(10**6))
