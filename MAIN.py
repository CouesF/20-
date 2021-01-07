#!/usr/bin/env python
import debugMode as a
import sys

PI = 3.1415926535

# before start
print("waitForStart\n")
a.waitForStart()
print("waitForStart\n")
a.resetToStartStatus()
a.genSerial.flush()

# start 

a.waitForStart()
a.camPos(1)
a.sleepFor(0.2)

#TODO: open new process

a.openListener()


##############################################################
a.lockMotors()
print("1\n")
a.setSpeed(0.75*PI,201,0)
while(abs(a.botCurGlobalPos[1] - 1.3) >= 0.01):#go to the 1 zone cam line
    pass
print("1\n")
a.lockMotors()
a.setSpeed(0.5*PI,201,0)
while(abs(a.botCurGlobalPos[0] - 3.0) >= 0.01):#getQRcode
    pass
print("1\n")
a.clawHeight(0)
a.sleepFor(0.1)
a.getAndSaveQrcode()
while(not a.decodeQRCode()):
    a.getAndSaveQrcode()
    pass
print(a.firstLevelOrder,a.secondLevelOrder)

a.clawDirection(0)
while(abs(a.botCurGlobalPos[0] - 6.8) >= 0.01):#match the color
    pass
a.unlockMotors()


sys.exit()
###################################
#color

if(a.firstLevelOrder[0]==0):
    a.clawHeight(1)
    a.claw(2)
    a.setMovement(0,6,0)
    a.sleepFor(1)
    a.claw(0)
    a.setMovement(0.,0)
    a.clawHeight(2)
    a.clawDirection(3)
    a.clawHeight(3)
    a.claw(1)
    a.clawHeight(2)
    a.clawDirection(0)

#a.setSpeed(0.5*PI,100,0)
while(abs(a.botCurGlobalPos[0] - 7.3) >= 0.01):#match the color
    pass
a.lockMotors()
#color
a.setSpeed(0.5*PI,100,0)
while(abs(a.botCurGlobalPos[0] - 7.8) >= 0.01):#match the color
    pass
a.lockMotors()
#color
a.setSpeed(0.5*PI,100,0)
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
while
a.setSpeed()
while(abs(x - tarX) > 0.01):
    pass
a.lockMotors()





a.set
setSpeed()



