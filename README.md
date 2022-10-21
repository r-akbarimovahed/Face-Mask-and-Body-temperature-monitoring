# Face-Mask-and-Body-temperature-monitoring
In this project, we implemented a face mask and body temperature monitoring system using raspberry pi 4 embedded system and convolutional neural networks techniques. Firstly, an ultrasound sensor measures the distance between the system and facing object. If the distance is between 10cm to 60cm, an image is acquired and the face region is detected. In the next step, the detected face is applied to a trained convolutional neural network model for classifying each face to mask and no mask classes. After that, the non-contact temperature sensor measures the body temperature from forehead. If the person is wearing a mask and his body temperature is less than 38.5 degrees, no alarm is activated, but if any of the above two situations occur, the system's alarm is activated.
# Software Dependencies 
OpenCV <br /> Tensorflow <br /> imutils <br /> Numpy <br /> screeninfo <br />  PIL <br /> skimage <br />  dlib <br />  adafruit_amg88xx <br /> board <br /> busio <br />
# Hardware Dependencies 
3.5inch RPi Display <br />
AMG8833 Non-contact tempreture sensor <br />
Raspberry Pi Camera <br /> 
Buzzer <br /> 
HC-SR04 <br /> 
# Instance result 
![Drawing5](https://user-images.githubusercontent.com/32155999/185734082-1ce59b0b-93b0-4a3d-a751-070ce77c366a.png)
# How to run 
1. run the Main_code.py in the direct of the project 
# Descriptions of file 
Main_code.py: The main code of the project <br />  
mask_detector.model: The trained CNN model for face mask classification <br />  
Slide1.png and Slide2.png: The images for guiding the user of the system <br />  
