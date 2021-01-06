#!/usr/bin/env python
#import cv2
import serial
import math
import time
import cv2
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout
import qrtools
pi = 3.141592653589793

clickX = 0
clickY = 0
templateWidth = 90

MKSDLC = '/dev/MKSDLC'
MKSGEN = '/dev/MKSGEN'

templateCam = '/dev/cam4TemplateMatching'
qrCodeCam = '/dev/cam4QRCode'

qrCodeCap = cv2.VideoCapture(qrCodeCam)
templateCap = cv2.VideoCapture(templateCam)

speed = [0, 0, 0, 0]# A B C D
botCurGlobalPos = [0,0,0] #x,y,dir float
firstLevelOrder = [0,0,0]
secondLevelOrder = [0,0,0]

baud = 115200
def openGen():
    genSerial = serial.Serial(
        port=MKSGEN,\
        baudrate=baud,\
        bytesize=serial.EIGHTBITS,\
        parity=serial.PARITY_NONE,\
        stopbits=serial.STOPBITS_ONE,\
        timeout=5)
    print(genSerial.name)         # check which port was really used
    pass
def openDlc():
    dlcSerial = serial.Serial(
        port=MKSDLC,\
        baudrate=baud,\
        bytesize=serial.EIGHTBITS,\
        parity=serial.PARITY_NONE,\
        stopbits=serial.STOPBITS_ONE,\
        timeout=5)
    print(dlcSerial.name)         # check which port was really used
    pass
    
def getTemplate():

    
    setMovement(0,-100,0) #move backward
    time.sleep(4)

    for i in range(1,4):
        frame = templateCap.read()
    
    cv2.imshow('image',frame)
    cv2.setMouseCallback("image", click_event)#https://stackoverflow.com/questions/28327020/opencv-detect-mouse-position-clicking-over-a-picture
    cv2.waitKey(0)
    template = frame[clickY-templateWidth:clickY+templateWidth,clickX-templateWidth:clickX+templateWidth]
    cv2.imshow('/home/coues/template/blueTemplate.png',template)
    cv2.imwrite(filename, image)
    cv2.waitKey(5)
    

    setMovement(0,-100,0)
    time.sleep(4)

    for i in range(1,4):
        frame = templateCap.read()
    w, h = template.shape[::-1] 
    matchingResult = cv2.matchTemplate(frame,template,cv2.TM_CCOEFF_NORMED) 
    min_v, max_v, min_pt, max_pt = cv2.minMaxLoc(matchingResult)
    frame = cv2.circle(frame, max_pt, 5, (255,0,0), 2)
    cv2.imshow('mactchingResult',frame)
    print('maxPosition:')
    print(max_pt)
    

    #3. cut and resize it


    #4. save it to the correct path
    cv2.waitKey(0)

    templateCap.release()
    cv2.destroyAllWindows()


def setSpeed(_direction,_speed,_rotation):
    getSpeed(_direction,_speed,_rotation)
    
    mesg = str.encode('S A'+str(int(speed[0]))+' B'+str(int(speed[1]))+' C'+str(int(speed[2]))+' D'+str(int(speed[3]))+' @')
    genSerial.write(mesg)
    print(mesg)

def setMovement(_x_,_y_,_r_):
    mesg = str.encode('P X'+str(int(_x_))+' Y'+str(int(_y_))+' R'+str(int(_r_))+'@')
    genSerial.write(mesg)
    print(mesg)

#getTemplate()

def waitUntilMovedTo(_x,_y,_dir):
    while(moveTo(_x,_y,_dir)!=0):
        pass
    pass





def test1():
    count = 3
    while(count>0):
    
        count=count-1

        time.sleep(1)

        setSpeed(-pi,80,0)
        time.sleep(1)
 
        setSpeed(-pi,120,0)
        time.sleep(2)

        setSpeed(-pi,30,0)
        time.sleep(1)


        time.sleep(2)
        setSpeed(0,0,0)
        print('set done\n')

   
    pass


def test4Template():
    cv2.waitKey(0)
    pass
def openAndSetCap():
   
    templateCap.set(10,0.8)#brightness
    #templateCap.set(15,10)#exposure
    #templateCap.set(11,0.3)#contrast
    print(templateCap.get(10),'\n')
    print(templateCap.get(11),'\n')
    print(templateCap.get(15),'\n')
    time.sleep(0.2)
    pass      
  
def getTemplatePosition():
    pass
def detectAndDecodeQRCode():#TODO:important,not todo
    '''
    requirement:open qrCodeCap
    output:set the 2 list of order
    '''
    getAndSaveQrcode()
    decodeQRCode()
    pass

#########################################
#test4Template()
openAndSetCap()

#openGen()
#test1()





#openListener()




#genSerial.close()
#dlcSerial.close()

templateCap.release()
qrCodeCap.release()

# while expression:
#     pass



# #1. Move backward for a fixed distance
# x = ser.read()          # read one byte
# s = ser.read(10)        # read up to ten bytes (timeout)
# ...     line = ser.readline()   # read a '\n' terminated line EOL

# #2. take a photo
# ret, frame = templateCap.read()
# cv2.imshow('test',ret)

# #3. cut and resize it


# #4. save it to the correct path
# cv2.waitKey(0)

# templateCap.release()
# cv2.destroyAllWindows()
