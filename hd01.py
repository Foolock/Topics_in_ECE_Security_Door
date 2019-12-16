from imutils.video import VideoStream
from imutils.video import FPS
import imutils
import time
import cv2
import numpy as np
import os
import RPi.GPIO as GPIO
import picamera


IR_Pin = 18
Trig_Pin = 23
Echo_Pin = 24
RED_Pin = 17
BLUE_Pin = 26
#VideoStream initial
def initial():
    vs = VideoStream(usePiCamera=True).start()
    time.sleep(1)
    fps = FPS().start()
def kill():
    fps.stop()
    cv2.destroyAllWindows()
    vs.stop()

#general_init
def init():
    GPIO.setwarnings(False)
    GPIO.setmode (GPIO.BCM)
    
    GPIO.setup(Trig_Pin, GPIO.OUT, initial = GPIO.LOW)
    GPIO.setup(Echo_Pin, GPIO.IN)
    GPIO.setup(IR_Pin, GPIO.IN)
    
    GPIO.setup(RED_Pin,GPIO.OUT, initial = GPIO.LOW)
    
    GPIO.setup(BLUE_Pin,GPIO.OUT)
    GPIO.output(BLUE_Pin,GPIO.HIGH)
    
    pass


def checkdist():
    GPIO.output(Trig_Pin, GPIO.HIGH)
    time.sleep(0.00015)
    GPIO.output(Trig_Pin, GPIO.LOW)
    while not GPIO.input(Echo_Pin):
        pass
    t1 = time.time()
    while GPIO.input(Echo_Pin):
        pass
    t2 = time.time()
    return (t2 - t1) * 340 * 100 / 2

def detect():
    while True:
        curtime = time.strftime('%Y-%m-%d-%H-%M-%S',time.localtime(time.time()))
        d = checkdist()
        if d <= 30 and GPIO.input(18) == True:                
            if recognize() == False:
                alert(curtime)
                camera = picamera.PiCamera()
                camera.capture('./unknown/' + curtime + '.jpg')
                camera.close()
            else:
                print('Unlock the door')
        else:
            continue
        time.sleep(3)

def alert(curtime):
        print (curtime + "Someone is coming!")
        #camera.capture(curtime + '.jpg')
        GPIO.setup(RED_Pin,GPIO.OUT)
        GPIO.output(RED_Pin,GPIO.HIGH)
        time.sleep(1)
        GPIO.output(RED_Pin,GPIO.LOW)

def recognize():
    detected = {}   # Record detected person and success times
    frames = 0      # Number of pictures taken
    rec_time = 0    # Detected times
    #initial()
    vs = VideoStream(usePiCamera=True).start()
    time.sleep(1)
    fps = FPS().start()
    while True:
        img = vs.read()
        img = imutils.resize(img, width=500)
        frames = frames + 1
        # camera.capture(rawCapture, format="bgr")
        # img = rawCapture.array

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        faces = faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.2,
            minNeighbors=5,
            minSize=(20, 20),
        )

        for (x, y, w, h) in faces:

            id, confidence = recognizer.predict(gray[y:y + h, x:x + w])

            probability = 100 - confidence
            # print('w, ',w,'h: ', h, 'p: ', probability)
            # Check if confidence is less them 100 ==> "0" is perfect match
            if (w < 160):
                cv2.putText(img, 'Please Move Closer', (250, 250), font, 1, (255, 255, 255), 2)
            elif(w > 250):
                cv2.putText(img, 'Please Move Further', (250, 250), font, 1, (255, 255, 255), 2)
            else:
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                if (probability > 20):
                    confidence = "  {0}%".format(round(probability))
                else:
                    # confidence = "  {0}%".format(round(100 - confidence))
                    id = 0
                    confidence = "{0}%".format(0)
                # Record detected person
                if id in detected:
                    detected[id] = detected[id] + 1
                    if id != 0 and detected[id] >= 10:
                        print('Welcome! ', names[id])
                        fps.stop()
                        cv2.destroyAllWindows()
                        vs.stop()
                        return True
                else:
                    detected[id] = 1
                rec_time = rec_time + 1
                if rec_time > 15:
                    print('Recognition Failed.')
                    fps.stop()
                    cv2.destroyAllWindows()
                    vs.stop()
                    return False
                name = names[id]
                cv2.putText(img, name, (x + 5, y - 5), font, 1, (255, 255, 255), 2)
                cv2.putText(img, str(confidence), (x + 5, y + h - 5), font, 1, (255, 255, 0), 1)

        cv2.imshow('camera', img)

        k = cv2.waitKey(10) & 0xff  # Press 'ESC' for exiting video
        if k == 27:
            break
        time.sleep(0.1)
        if frames > 30:
            print('Recognition Failed. Undetected!')
            fps.stop()
            cv2.destroyAllWindows()
            vs.stop()
            return False
        fps.update()

#facial_dect_initial-----------------------
recognizer = cv2.face.LBPHFaceRecognizer_create()
recognizer.read('trainer/trainer.yml')
cascadePath = "haarcascade_frontalface_default.xml"
faceCascade = cv2.CascadeClassifier(cascadePath)
font = cv2.FONT_HERSHEY_SIMPLEX
#iniciate id counter
id = 0
# names related to ids
names = ['Unknown', 'Jimmy', 'Zhuoran', 'Abbie'] 
time.sleep(2)
#------------------------------------------

#general initial---------------------
init()  #func1

# Define min window size to be recognized as a face
detect()  #func2
 # stop the timer and display FPS information

# Do a bit of cleanup
GPIO.cleanup()
