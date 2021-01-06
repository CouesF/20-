#!/usr/bin/env python
# main program
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

######################################TODO:
materialPosition = np.zeros((4, 3, 3), dtype=float)#area-color-xydir







#####################################TODO:

def callback(rawPositionData):
    #rospy.loginfo(rospy.get_caller_id() + "i heard %s", rawPositionData.data)
    botCurGlobalPos[0] = rawPositionData.data[0]
    botCurGlobalPos[1] = rawPositionData.data[1]
    botCurGlobalPos[2] = rawPositionData.data[2]
    #print(botCurGlobalPos)

def openListener():
    rospy.init_node('mainPython',anonymous=True)
    rospy.Subscriber('RobotPositionInfo',Float32MultiArray,callback)
    #rospy.spin()


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


    
def getTemplate(color):
    for i in range(1,4):
        _,frame = templateCap.read()
    cv2.imshow('image',frame)
    cv2.setMouseCallback("image", click_event)#https://stackoverflow.com/questions/28327020/opencv-detect-mouse-position-clicking-over-a-picture
    cv2.waitKey(0)
    template = frame[clickY-templateWidth:clickY+templateWidth,clickX-templateWidth:clickX+templateWidth]
    cv2.imshow('template',template)
    cv2.imwrite('/home/coues/template/blueTemplate.png',template)
    cv2.waitKey(0)
    
def getTemplatePos():
    for i in range(1,4):
        _,frame = templateCap.read()
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

def readTemplate():
    redTemplate = imread('/home/coues/template/redTemplate.png')
    greenTemplate = imread('/home/coues/template/greenTemplate.png')
    blueTemplate = imread('/home/coues/template/blueTemplate.png')
    pass

def setSpeed(_direction,_speed,_rotation):
    getSpeed(_direction,_speed,_rotation)
    
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

#getTemplate()
def moveTo(targetX,targetY,targetDir):#will sent one record of moving data
    maxSpeed = 100
    returnValue = 0
    deltaX = targetX - botCurGlobalPos[0]
    deltaY = targetY - botCurGlobalPos[1]
    deltaRotation = targetDir - botCurGlobalPos[2]
    
    moveDir = math.atan2(deltaY,deltaX)
    moveDir += pi / 4.0
    
    dist = math.sqrt(math.pow(deltaX,2)+math.pow(deltaY,2))
    if(dist > 1.2):
        movingSpeed = maxSpeed * 0.9
        returnValue += 1
    elif(dist>0.15):
        movingSpeed = maxSpeed * dist/1.2
        returnValue += 2
    elif(dist>0.08):
        movingSpeed = maxSpeed * 0.15/1.2
        returnValue += 3
    else:
        movingSpeed = 0

    if(abs(deltaRotation) > pi/2):
        rotationSpeed = maxSpeed * 0.45
        returnValue +=10
    elif(abs(deltaRotation) > pi/8):
        rotationSpeed = maxSpeed * 0.3
        returnValue +=20
    elif(abs(deltaRotation) > pi/15):
        rotationSpeed = maxSpeed * 0.1
        returnValue +=30
    else:
        rotationSpeed = 0
    print(moveDir,movingSpeed,deltaRotation,'\n')
    rotationSpeed *=np.sign(deltaRotation)
    setSpeed(moveDir,movingSpeed,rotationSpeed)

    return returnValue

    pass

def waitUntilMovedTo(_x,_y,_dir):
    while(moveTo(_x,_y,_dir)!=0):
        pass
    pass



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
def getAndSaveQrcode():
    _,imgggg =qrCodeCap.read()
    cv2.imshow('qrcode',imgggg)
    cv2.imwrite("qrcode.png",imgggg)
    pass
def decodeQRCode():
    qr = qrtools.QR()
    qr.decode("qrcode.png")
    print(qr.data)
    print(qr.data[0])
    
    for i in range(0,3):
        firstLevelOrder[i] = int(qr.data[i])
        secondLevelOrder[i] = int(qr.data[i+4])
        pass
    print(firstLevelOrder,secondLevelOrder)
    return qr.data

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
