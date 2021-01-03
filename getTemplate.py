#import cv2
import serial
import math
import time
import cv2
import numpy as np
pi = 3.141592653589793

clickX = 0
clickY = 0
templateWidth = 90

MKSGEN = '/dev/MKSGEN'
templateCam = '/dev/cam4TemplateMatching'
baud = 115200
genSerial = serial.Serial(
    port=MKSGEN,\
    baudrate=baud,\
    bytesize=serial.EIGHTBITS,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    timeout=5)

print(genSerial.name)         # check which port was really used
speed = [0, 0, 0, 0]# A B C D



def getSpeed(direction, speedG, rotation):
    """
    docstring
    """
    direction = direction + (pi / 4.0)
    speed[0] = -speedG * math.sin(direction)
    speed[1] = speedG * math.cos(direction)
    speed[2] = speedG * math.sin(direction)
    speed[3] = -speedG * math.cos(direction)

    speed[0] += rotation
    speed[1] += rotation
    speed[2] += rotation
    speed[3] += rotation
    pass

def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        clickX = x
        clickY = y
        print(clickX,",",clickY)
        font = cv2.FONT_HERSHEY_SIMPLEX
        strXY = str(x)+", "+str(y)
        cv2.putText(frame, strXY, (x,y), font, 0.5, (255,255,0), 2)
        cv2.imshow("image", frame)
        


    
def getTemplate():

    cam = cv2.VideoCapture(templateCam)
    
    setMovement(0,-100,0) #move backward
    time.sleep(4)

    for i in range(1,4):
        frame = cam.read()
    
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
        frame = cam.read()
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

    cam.release()
    cv2.destroyAllWindows()


def setSpeed():
    mesg = str.encode('S A'+str(int(speed[0]))+' B'+str(int(speed[1]))+' C'+str(int(speed[2]))+' D'+str(int(speed[3]))+' @')
    genSerial.write(mesg)
    print(mesg)

def setMovement(x,y,r):
    mesg = str.encode('P X'+str(int(x))+' Y'+str(int(y))+' R'+str(int(r))+' @')
    genSerial.write(mesg)
    print(mesg)
#genSerial.write(str.encode('bnbs'))
#xxx = genSerial.read(5)
#print((xxx))
#temp = str.encode('S A50 B50 C50 D50 ')
#genSerial.write(temp)
#print(temp)

time.sleep(1)

getSpeed(-pi,80,0)
setSpeed()
time.sleep(8)

getSpeed(-pi,120,0)
setSpeed()
time.sleep(3)

getSpeed(-pi,30,0)
setSpeed()
time.sleep(2)


getSpeed(0,0,0)
time.sleep(8)
setSpeed()
print('set done\n')

genSerial.close()


# while expression:
#     pass



# #1. Move backward for a fixed distance
# x = ser.read()          # read one byte
# s = ser.read(10)        # read up to ten bytes (timeout)
# ...     line = ser.readline()   # read a '\n' terminated line EOL

# #2. take a photo
# ret, frame = cam.read()
# cv2.imshow('test',ret)

# #3. cut and resize it


# #4. save it to the correct path
# cv2.waitKey(0)

# cam.release()
# cv2.destroyAllWindows()