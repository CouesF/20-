#import cv2
import serial
import math
import time
pi = 3.141592653589793

MKSGEN = '/dev/MKSGEN'
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
templateCam = '/dev/templateCam'
#cam = cv2.VideoCapture(templateCam)

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

def setSpeed():
    mesg = str.encode('S A'+str(int(speed[0]))+' B'+str(int(speed[1]))+' C'+str(int(speed[2]))+' D'+str(int(speed[3]))+'  ')
    genSerial.write(mesg)
    print( mesg)

#genSerial.write(str.encode('bnbs'))
#xxx = genSerial.read(5)
#print((xxx))
time.sleep(2)
getSpeed(-pi,70,0)
setSpeed()
time.sleep(8)
getSpeed(0,0,0)
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
