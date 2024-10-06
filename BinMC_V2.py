import threading
import time
import cv2 as cv
import json
import os
from flask_cors import CORS
from flask import Flask, request, jsonify, Response
from flask_socketio import SocketIO, emit
import asyncio
from gpiozero import LED, PWMLED, DistanceSensor
from ultralytics import YOLO
import speech_recognition as sr
import websockets
import numpy as np
import subprocess
import math
from adafruit_servokit import ServoKit
import random
import board
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219
import busio
import adafruit_vl53l0x
from digitalio import DigitalInOut
from adafruit_vl53l0x import VL53L0X
import pygame
import mediapipe as mp
from pydub import AudioSegment
from io import BytesIO
import RPi.GPIO as GPIO
from mpu6050 import mpu6050

from Def import MotorDrive,Gyro,VarGlobal,GetImg


def main():
    while True:
        print(VarGlobal.yaw)
        time.sleep(0.1)

if __name__ == "__main__":
    MotorDrive.SetUp4Pin(pin_fr=22,pin_bl=17,pin_br=27,pin_fl=4)
    Gyro.Setup_Gyro()
    
    Gyro_thread = threading.Thread(target=Gyro.get_gyro)
    Main_Program = threading.Thread(target=main)
    Get_Img = threading.Thread(target=GetImg.read_camera)
    
    Gyro_thread.daemon = True
    Main_Program.daemon = True
    Get_Img.daemon = True
    
    Get_Img.start()
    Gyro_thread.start()
    Main_Program.start()
    
    Gyro_thread.join()
    Main_Program.join()
    Get_Img.join()

