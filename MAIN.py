#!/usr/bin/env python
import debugMode as a
import sys
from subprocess import call,Popen


PI = 3.1415926535
zone1PositionX=[6.8,7.3,7.8]
firstLayerColor=[0,0,0]
order=0

# before start
print("waitForStart\n")
a.waitForStart()

a.resetToStartStatus()
a.genSerial.flush()

# start 
print("waitForStart\n")
a.waitForStart()
a.camPos(1)
Popen('./../../../home/coues/StartLoc.sh')
a.sleepFor(1)

#TODO: open new process

a.openListener()






##############################################################
a.lockMotors()


#########################
a.setSpeed(0.75*PI,231,0)
while(a.botCurGlobalPos[1] <  1.1):# go to the zone1 x-axis
    pass
print("1\n")
a.lockMotors()
a.sleepFor(1)
########################
a.setSpeed(0.5*PI,300,0)
while(a.botCurGlobalPos[0] < 3):# getQRcode
    pass
print("1\n")
a.clawHeight(0)

#####################QRCODE
print('qrCode')
a.getAndSaveQrcode()
hasQRCode = a.decodeQRCode()
while(not hasQRCode):
    print('qrCode')
    a.sleepFor(0.35)
    a.getAndSaveQrcode()
    hasQRCode = a.decodeQRCode()
    pass
print(a.firstLevelOrder,a.secondLevelOrder)



a.clawDirection(0)
########################
while(a.botCurGlobalPos[0] >= zone1PositionX[0]):#match the color
    pass
if(a.firstLevelOrder[order]==firstLayerColor[0]):
    a.preCatch(1)
    a.putToStorage(0)
    order=1
a.setSpeed(0.5*PI,150,0)

while(a.botCurGlobalPos[0] >=zone1PositionX[1]):#match the color
    pass
if(a.firstLevelOrder[order]==firstLayerColor[1]):
    a.preCatch(1)
    a.putToStorage(order)
    order=+=1

if(order==0):
    a.preCatch(1)
    a.putToStorage(order)
    order=1
    for i in range(0,1):
        if(a.firstLevelOrder[j]==firstLayerColor[i]):
            targetItem=i
    a.setSpeed(-0.5*PI,150,0)
    while(a.botCurGlobalPos[0] <= zone1PositionX[targetItem]):
        a.preCatch(1)
        a.putToStorage(1)
        order=2
    if(targetItem==0):
        a.setSpeed(0.5*PI,150,0)
        while(a.botCurGlobalPos[0] <= zone1PositionX[0]):
    else:
        a.setSpeed(-0.5*PI,150,0)
        while(a.botCurGlobalPos[0] >= zone1PositionX[0]):
    a.preCatch(1)
    a.putToStorage(2)
    order=0
elif(order==1):
    for i in range(0,2):
        if(a.firstLevelOrder[j]==firstLayerColor[i]):
            targetItem=i
    if(targetItem>1):
        a.setSpeed(0.5*PI,150,0)
        while(a.botCurGlobalPos[0] >= zone1PositionX[targetItem]):
    else:
        a.setSpeed(-0.5*PI,150,0)
        while(a.botCurGlobalPos[0] <= zone1PositionX[targetItem]):
    a.preCatch(1)
    a.putToStorage(1)
    order=2
    formerItem=targetItem
    for i in range(0,2):
        if(a.firstLevelOrder[j]==firstLayerColor[i]):
            targetItem=i
    if(targetItem>formerItem0):
        a.setSpeed(0.5*PI,150,0)
        while(a.botCurGlobalPos[0] >= zone1PositionX[targetItem]):
    else:
        a.setSpeed(-0.5*PI,150,0)
        while(a.botCurGlobalPos[0] <= zone1PositionX[targetItem]):
    a.preCatch(1)
    a.putToStorage(2)
    order=0
else:
    a.setSpeed(0.5*PI,150,0)
    while(a.botCurGlobalPos[0] >= zone1PositionX[2]):
    a.preCatch(1)
    a.putToStorage(2)
    order=0


    

while(abs(a.botCurGlobalPos[1] - 3.26) >= 0.01):#go to the rotating corner
    pass
while(abs(a.botCurGlobalPos[2] - 0.04) >= 0.01):#rotate towards the 2 zone
    pass
while(abs(a.botCurGlobalPos[0] - 7) >= 0.01):#go to the 2 zone cam line
    pass
while(abs(a.botCurGlobalPos[1] - 3.26) >= 0.01):#go the R Position
    pass
while(abs(a.botCurGlobalPos[1] - 3.67) >= 0.01):#go the G Position
    pass
while(abs(a.botCurGlobalPos[1] - 4.2) >= 0.01):#go the B Position
    pass
while(abs(a.botCurGlobalPos[0] - 4.89) >= 0.01):#go to the rotating corner
    pass
while(abs(a.botCurGlobalPos[2] - 1.63) >= 0.01):#rotate towards the 3 zone
    pass
while(abs(a.botCurGlobalPos[1] - 4.7) >= 0.01):#go to the 3 zone cam line
    pass
while(abs(a.botCurGlobalPos[0] - 4.89) >= 0.01):#go the R Position
    pass
while(abs(a.botCurGlobalPos[0] - 4.44) >= 0.01):#go the G Position
    pass
while(abs(a.botCurGlobalPos[0] - 3.92) >= 0.01):#go the B Position
    pass
while(abs(a.botCurGlobalPos[1] - 3) >= 0.01):#go to the rotating corner
    pass
while(abs(a.botCurGlobalPos[2] - -1.59) >= 0.01):#rotate towards the 2 zone
    pass
while(abs(a.botCurGlobalPos[0] - 3.0) >= 0.01):#go to the 1 zone line
    pass
while(abs(a.botCurGlobalPos[0] - 6.8) >= 0.01):#fetch the target object
    pass
while(abs(a.botCurGlobalPos[0] - 7.3) >= 0.01):#fetch the target object
    pass
while(abs(a.botCurGlobalPos[0] - 7.8) >= 0.01):#fetch the target object
    pass
while(abs(a.botCurGlobalPos[1] - 3.26) >= 0.01):#go to the rotating corner
    pass
while(abs(a.botCurGlobalPos[2] - 0.04) >= 0.01):#rotate towards the 2 zone
    pass
while(abs(a.botCurGlobalPos[0] - 7) >= 0.01):#go to the 2 zone cam line
    pass
while(abs(a.botCurGlobalPos[1] - 3.26) >= 0.01):#go the R Position
    pass
while(abs(a.botCurGlobalPos[1] - 3.67) >= 0.01):#go the G Position
    pass
while(abs(a.botCurGlobalPos[1] - 4.2) >= 0.01):#go the B Position
    pass
while(abs(a.botCurGlobalPos[0] - 4.89) >= 0.01):#go to the rotating corner
    pass
while(abs(a.botCurGlobalPos[2] - 1.63) >= 0.01):#rotate towards the 3 zone
    pass
while(abs(a.botCurGlobalPos[1] - 4.7) >= 0.01):#go to the 3 zone cam line
    pass
while(abs(a.botCurGlobalPos[0] - 4.89) >= 0.01):#go the R Position
    pass
while(abs(a.botCurGlobalPos[0] - 4.44) >= 0.01):#go the G Position
    pass
while(abs(a.botCurGlobalPos[0] - 3.92) >= 0.01):#go the B Position
    pass
while(abs(a.botCurGlobalPos[0] - 0) >= 0.01):
    pass
while(abs(a.botCurGlobalPos[1] - 6) >= 0.01):
    pass
a.setSpeed()
while(abs(x - tarX) > 0.01):
    pass
a.lockMotors()








