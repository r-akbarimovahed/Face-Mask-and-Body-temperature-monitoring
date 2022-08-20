# Face-Mask-and-Body-temperature-monitoring
In this project, we implemented a face mask and body temperature monitoring system using raspberry pi 4. Firstly, an ultrasound sensor measures the distance between the system and facing object. If the distance is between 10cm to 60cm, an image is acquised and the face region is detected. In the next step, the detected face is applied to a trained CNN model for classifying each face to mask and no mask classes. After that, the non-contact tempereture sensor measures the body thempreture from forehead. If the person is wearing a mask and his body temperature is less than 38.5 degrees, no alarm is activated, but if any of the above two situations occur, the system's alarm is activated.
# Software Dependencies 
OpenCV <br /> Tensorflow <br /> imutils <br /> Numpy <br /> screeninfo <br />  PIL <br /> skimage <br />  dlib <br />  adafruit_amg88xx <br /> board <br /> busio <br />
# Hardware Dependencies 
3.5inch RPi Display <br />
AMG8833 Non-contact tempreture sensor <br />
Raspberry Pi Camera <br /> 
Buzzer <br /> 
HC-SR04 <br /> 

