import cv2
import serial

MKSGEN = '/dev/MKSGEN'
baud = 115200
genSerial = serial.Serial(MKSGEN,baud,timeout = 1)
print(genSerial.name)         # check which port was really used

templateCam = '/dev/templateCam'
cam = cv2.VideoCapture(templateCam)

while expression:
    pass



#1. Move backward for a fixed distance
x = ser.read()          # read one byte
s = ser.read(10)        # read up to ten bytes (timeout)
...     line = ser.readline()   # read a '\n' terminated line

#2. take a photo
ret, frame = cam.read() 
cv2.imshow('test',ret)

#3. cut and resize it


#4. save it to the correct path
cv2.waitKey(0)

cam.release()
cv2.destroyAllWindows() 
