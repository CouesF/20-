import cv2
import serial
import math

pi = 3.141592653589793

MKSGEN = '/dev/MKSGEN'
baud = 115200
genSerial = serial.Serial(
    port=MKSGEN,\
    baudrate=9600,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)
print(genSerial.name)         # check which port was really used
speed = [0, 0, 0, 0]# A B C D
templateCam = '/dev/templateCam'
#cam = cv2.VideoCapture(templateCam)

def getSpeed(dir, speedG, rotation)
    """
    docstring
    """
    dir = dir + (pi / 4.0)
    speed[0] = -speedG * math.sin(dir)
    speed[1] = speedG * math.cos(dir)
    speed[2] = speedG * math.sin(dir)
    speed[3] = -speedG * math.cos(dir)

    speed[0] += rotation
    speed[1] += rotation
    speed[2] += rotation
    speed[3] += rotation
    pass

def setSpeed()
    genSerial.write('S A'+speed[0]+' B'+speed[1]+' C'+speed[2]+' D'+speed[3]+'\n')

getSpeed(-pi,50,0)
setSpeed()
time.sleep(1)
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
