# -*- coding: utf-8 -*-
"""
Created on Tue Nov 17 20:28:30 2020

@author: Reza
"""

from tensorflow.keras.applications.mobilenet_v2 import preprocess_input
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
from imutils.video import VideoStream
import numpy as np
import imutils
import time
import cv2
import os
import RPi.GPIO as GPIO
import screeninfo
from PIL import Image, ImageDraw, ImageFont
from skimage import exposure

import busio
import board
import adafruit_amg88xx

import dlib 

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False) 
#set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 16
BUZZER = 19


GPIO.setup(BUZZER, GPIO.OUT) 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)


i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)
detector = dlib.get_frontal_face_detector() 
Img_Img_Acq = cv2.imread(os.path.join(os.getcwd(),'Slide1.png') ,cv2.IMREAD_UNCHANGED)
Img_Temp_Acq = cv2.imread(os.path.join(os.getcwd(),'Slide2.png'), cv2.IMREAD_UNCHANGED)

fnt = ImageFont.truetype('Lato-Medium.ttf',35)

screen = screeninfo.get_monitors()[0]
width, height = screen.width , screen.height

def IMSHOW(Time_delay,frame,screen,width, height, C):
    cv2.namedWindow('Frame', cv2.WND_PROP_FULLSCREEN)
    cv2.moveWindow('Frame', screen.x-1, screen.y-1)
    cv2.setWindowProperty('Frame', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow("Frame", frame)
    cv2.waitKey(Time_delay)
    if C:
        cv2.destroyAllWindows()

def Get_Temp():
    a = np.asarray(amg.pixels)
    a = np.sort(a,axis=None)
    a = a[a.shape[0]-10:-1]
    Temp =  (0.341*a.mean()) + 26.69
    return Temp

def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance =round((TimeElapsed * 34300) / 2)
    return distance
#     
#     if 20<distance<100:
#         
#         return distance
#     else:
#         return 0

def Face_Detection(frame,detector):
	# grab the dimensions of the frame and then construct a blob
	# from it
    frame1 = frame.copy()
    
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = exposure.adjust_gamma(frame, gamma=0.5, gain=1)
    faces = detector(gray)
    Faces = []
    locs = []
    for face in faces:
        x1 = face.left()
        y1 = face.top()
        x2 = face.right()
        y2 = face.bottom()
        box = (x1, y1 , x2, y2)
        face = frame[y1:y2 ,x1:x2,:]
        face = cv2.cvtColor(face, cv2.COLOR_BGR2RGB)
        Faces.append(face)
        locs.append(box)
        cv2.rectangle(frame1, (x1,y1), (x2,y2), (255,255,0),4)
    return frame1, Faces, locs
 

def detect_and_predict_mask(FACES, maskNet):
	# grab the dimensions of the frame and then construct a blob
	# from it
    preds = []   
    All_F = []
    for i in range(len(FACES)):
        face = Faces[i]
        face = cv2.resize(face, (224, 224))
        face = img_to_array(face)
        face = preprocess_input(face)
        All_F.append(face)
        if len(All_F)>0:
            All_F = np.array(All_F, dtype="float32")
            preds = maskNet.predict(All_F, batch_size=32)
    return preds

def Image_captioning(locs,preds,frame,Temp):
    for (box, pred) in zip(locs, preds):
        # unpack the bounding box and predictions
        (startX, startY, endX, endY) = box
        (mask, withoutMask) = pred

        # determine the class label and color we'll use to draw
        # the bounding box and text
        label = "Mask" if mask > withoutMask else "No Mask"
        color = (0, 255, 0) if label == "Mask" else (0, 0, 255)
        Prediction = label

        # include the probability in the label
        # label = "{}: {:.2f}%".format(label, max(mask, withoutMask) * 100)
        label = "{}: {:.2f}% __{}: {}".format(label, max(mask, withoutMask) * 100,'Temp', round(Temp,2))

        # display the label and bounding box rectangle on the output
        # frame
        cv2.putText(frame, label, (startX, startY - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
        cv2.rectangle(frame, (startX, startY), (endX, endY), color, 2)
    return frame, Prediction

def buzz(noteFreq, duration):
    halveWaveTime = 1 / (noteFreq * 2 )
    waves = int(duration * noteFreq)
    for i in range(waves):
       GPIO.output(BUZZER, True)
       time.sleep(halveWaveTime)
       GPIO.output(BUZZER, False)
       time.sleep(halveWaveTime)

def play(Label):
    t=0
#      notes=[262,294,330,262,262,294,330,262,330,349,392,330,349,392,392,440,392,349,330,262,392,440,392,349,330,262,262,196,262,262,196,262]
    notes=[]
    duration=[]
    if Label==True:
        for i in range(32):
            notes.append(5)
            duration.append(0.20)
    else:
        notes.append(5)
        duration.append(0.20)
#     duration=[0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,1,0.5,0.5,1,0.25,0.25,0.25,0.25,0.5,0.5,0.25,0.25,0.25,0.25,0.5,0.5,0.5,0.5,1,0.5,0.5,1]
    for n in notes:
        buzz(n, duration[t])
        time.sleep(duration[t] *0.1)
        t+=1
        

def Temp_Captioning(fnt, Temp, frame, label):
    if (label=='Mask')and(Temp<38):
        image = Image.new(mode = "RGB", size = (200,300), color = "green")
        sound= False
#         play(False)
    else:
        image = Image.new(mode = "RGB", size = (200,300), color = "red")
        sound= True
#         play(True)
    draw = ImageDraw.Draw(image)
    Text = "{}{:.2f}".format('Temp:', Temp)
    draw.text((5,150), Text, font=fnt, fill=(255,255,0))
    draw.text((5,200), label, font=fnt, fill=(255,255,0))
    I = np.asarray(image)
    I = cv2.cvtColor(I,cv2.COLOR_RGB2BGR)
    final_frame = cv2.hconcat([frame,I])
    return final_frame,sound
    
FILE_PATH=os.getcwd() + "/"
# face_path=FILE_PATH+r"face_detector"

# load the face mask detector model from disk
print("[INFO] loading face mask detector model...")
maskNet = load_model(os.path.join(os.getcwd(),'mask_detector.model'))
# 
# # initialize the video stream and allow the camera sensor to warm up
# print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
Temp = Get_Temp()

while(True):

    Dis = distance()
    time.sleep(0.5)
    print(Dis)

    if 10<Dis<60:
        Locs = []
        All_Faces = []
        play(False)
        IMSHOW(5*1000,Img_Img_Acq,screen,width, height, True)
        for i in range(1000):
            frame = vs.read()
            frame = imutils.resize(frame, width=400)
            frame_face_detected, Faces, locs = Face_Detection(frame,detector)
            Locs.append(locs)
            IMSHOW(int(0.15*1000),frame_face_detected,screen,width, height,False)
            All_Faces.append(Faces)
            if (len(Faces)>=1):
                break
        cv2.destroyAllWindows()        
            
        play(False)
        # detect faces in the frame and determine if they are wearing a
        # face mask or not
        preds = detect_and_predict_mask(All_Faces[-1],maskNet)
        #print('Please bring your forehead closer to the device')
        #time.sleep(5.0)
        IMSHOW(5*1000,Img_Temp_Acq,screen,width, height, True)
        try:
            Temp = Get_Temp()
            play(False)
            frame, label = Image_captioning(Locs[-1],preds,frame,Temp)
            frame,sound = Temp_Captioning(fnt, Temp, frame, label)
            IMSHOW(1*1000,frame,screen,width, height, True)
            play(sound)
            IMSHOW(5*1000,frame,screen,width, height,True)
        except:     
            IMSHOW(1*1000,frame,screen,width, height,True)
