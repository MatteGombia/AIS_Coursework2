#!/usr/bin/env python3

import sys

import math
from frame2d import Frame2D
# you can experiment with any of your own data recordings by importing them here
from drive_test_1 import robotFrames
#from drive_test_2_cliff import robotFrames
import numpy

if len(sys.argv) == 2:
	logName = sys.argv[1]+".py"
else:
	logName = "w7_robotLog.py"

velocityFrames = []
sensorFrames = []
sensorNoiseX = 0.01
sensorNoiseY = 0.01
sensorNoiseA = 0.0001
sensorNoiseVar = numpy.diag([sensorNoiseX, sensorNoiseY, sensorNoiseA])

lastXYAFrame = robotFrames[0][1]
for t, frame in robotFrames:
    deltaA = frame.angle()-lastXYAFrame.angle()
    if deltaA > 2*math.pi:
       deltaA = deltaA % 2*math.pi
    if deltaA < -2*math.pi:
       deltaA = -(deltaA % 2*math.pi)
    velocityFrames.append((t, Frame2D.fromXYA(frame.x()-lastXYAFrame.x(),frame.y()-lastXYAFrame.y(),deltaA)))
    sensorRandomised = numpy.random.multivariate_normal(numpy.array([frame.x(), frame.y(), frame.angle()]),
                                                 sensorNoiseVar)
    sensorFrames.append((t,Frame2D.fromXYA(sensorRandomised[0], sensorRandomised[1], sensorRandomised[2])))
    lastXYAFrame = frame
    
logFile = open(logName, 'w')
print("from frame2d import Frame2D", file=logFile)
print("robotFrames = [", file=logFile)
idx = 0
for rt, rf in robotFrames:
    print("   (%d, Frame2D.fromXYA(%f,%f,%f))" % (rt,rf.x(),rf.y(),rf.angle()), end="", file=logFile)
    if idx != len(robotFrames)-1:
       print(",", file=logFile)
    idx += 1
print("]", file=logFile)
print("sensorFrames = [", file=logFile)
idx = 0
for st, sf in sensorFrames:
    idx
    print("   (%d, Frame2D.fromXYA(%f,%f,%f))" % (st,sf.x(),sf.y(),sf.angle()), end="", file=logFile)
    if idx != len(sensorFrames)-1:
       print(",", file=logFile)
    idx += 1
print("]", file=logFile)
print("motionFrames = [", file=logFile)
idx = 0
for vt, vf in velocityFrames:
    print("   (%d, Frame2D.fromXYA(%f,%f,%f))" % (vt,vf.x(),vf.y(),vf.angle()), end="", file=logFile)
    if idx != len(velocityFrames)-1:
       print(",", file=logFile)
    idx += 1
print("]", file=logFile)
logFile.close()

