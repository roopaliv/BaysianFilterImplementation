#!/usr/bin/env python
import sys
import rospy
import rosbag
from lab4.msg import Motion
from lab4.msg import Observation
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import random
import math
import tf

tag0x = 1.25;
tag0y = 5.25;
tag1x = 1.25;
tag1y = 3.25;
tag2x = 1.25;
tag2y = 1.25;
tag3x = 4.25;
tag3y = 1.25;
tag4x = 4.25;
tag4y = 3.25;
tag5x = 4.25;
tag5y = 5.25;
tags = {0: (tag0x, tag0y), 1: (tag1x, tag1y), 2: (tag2x, tag2y),3: (tag3x, tag3y),4: (tag4x, tag4y),5: (tag5x, tag5y)}

occupancyGrid = [[[0 for x in range(35)] for y in range(35)] for t in range(36)]
lt = []
lx = []
ly = []
ObservedBoxX = 11
ObservedBoxY= 27
ObservedBoxT = int(math.ceil(200.52/10))
ObservedPoseX = (11*0.2)+0.1
ObservedPoseY= (27*0.2)+0.1
ObservedPoseT = 200.52

def distributeProbabilities():
    global occupancyGrid, ObservedBoxT, ObservedBoxY, ObservedBoxT
    minusT = (ObservedBoxT-1 >= 0)
    plusT = (ObservedBoxT+1 <= 35)
    minusX = (ObservedBoxX - 1 >=0)
    plusX = (ObservedBoxX + 1 <=34)
    minusY = (ObservedBoxY - 1 >=0)
    plusY = (ObservedBoxY + 1 <=34)
    #rospy.logerr(str(ObservedBoxT)+ " " + str(ObservedBoxY)+ " " + str(ObservedBoxX))
    temp = 1 if (occupancyGrid[ObservedBoxT][ObservedBoxY][ObservedBoxX]== 0) else (
        occupancyGrid[ObservedBoxT][ObservedBoxY][ObservedBoxX])
    occupancyGrid[ObservedBoxT][ObservedBoxY][ObservedBoxX] = temp * (30/100)
    if(minusT):
        temp = 1 if (occupancyGrid[ObservedBoxT-1][ObservedBoxY][ObservedBoxX]== 0) else (
            occupancyGrid[ObservedBoxT-1][ObservedBoxY][ObservedBoxX])
        occupancyGrid[ObservedBoxT-1][ObservedBoxY][ObservedBoxX] = temp * (15 / 100)
    if(plusT):
        temp = 1 if (occupancyGrid[ObservedBoxT+1][ObservedBoxY][ObservedBoxX]== 0) else (
            occupancyGrid[ObservedBoxT+1][ObservedBoxY][ObservedBoxX])
        occupancyGrid[ObservedBoxT+1][ObservedBoxY][ObservedBoxX] = temp * (15 / 100)


    if (minusX):
        temp = 1 if (occupancyGrid[ObservedBoxT][ObservedBoxY][ObservedBoxX - 1]== 0) else (
            occupancyGrid[ObservedBoxT][ObservedBoxY][ObservedBoxX - 1])
        occupancyGrid[ObservedBoxT][ObservedBoxY][ObservedBoxX - 1] = temp * (2.5 / 100)
        if (minusT):
            temp = 1 if (occupancyGrid[ObservedBoxT - 1][ObservedBoxY][ObservedBoxX - 1] == 0) else (
            occupancyGrid[ObservedBoxT - 1][ObservedBoxY][ObservedBoxX - 1] == 0)
            occupancyGrid[ObservedBoxT - 1][ObservedBoxY][ObservedBoxX - 1] = temp * (1.25 / 100)
        if (plusT):
            temp = 1 if (occupancyGrid[ObservedBoxT + 1][ObservedBoxY][ObservedBoxX - 1] == 0) else (
            occupancyGrid[ObservedBoxT + 1][ObservedBoxY][ObservedBoxX - 1])
            occupancyGrid[ObservedBoxT + 1][ObservedBoxY][ObservedBoxX - 1] = temp * (1.25 / 100)

    if (plusX):
        temp = 1 if (occupancyGrid[ObservedBoxT][ObservedBoxY][ObservedBoxX + 1]== 0) else(
            occupancyGrid[ObservedBoxT][ObservedBoxY][ObservedBoxX + 1])
        occupancyGrid[ObservedBoxT][ObservedBoxY][ObservedBoxX + 1] = temp * (2.5 / 100)
        if (minusT):
            temp = 1 if (occupancyGrid[ObservedBoxT - 1][ObservedBoxY][ObservedBoxX + 1] == 0) else (
                occupancyGrid[ObservedBoxT - 1][ObservedBoxY][ObservedBoxX + 1])
            occupancyGrid[ObservedBoxT - 1][ObservedBoxY][ObservedBoxX + 1] = temp * (1.25 / 100)
        if (plusT):
            temp = 1 if (occupancyGrid[ObservedBoxT + 1][ObservedBoxY][ObservedBoxX + 1] == 0) else (
                occupancyGrid[ObservedBoxT + 1][ObservedBoxY][ObservedBoxX + 1])
            occupancyGrid[ObservedBoxT + 1][ObservedBoxY][ObservedBoxX + 1] = temp * (1.25 / 100)

    if(minusY):
        temp = 1 if (occupancyGrid[ObservedBoxT][ObservedBoxY-1][ObservedBoxX]== 0) else(
            occupancyGrid[ObservedBoxT][ObservedBoxY-1][ObservedBoxX])
        occupancyGrid[ObservedBoxT][ObservedBoxY-1][ObservedBoxX] = temp * (2.5 / 100)
        if (minusT):
            temp = 1 if (occupancyGrid[ObservedBoxT - 1][ObservedBoxY-1][ObservedBoxX] == 0) else (
                occupancyGrid[ObservedBoxT - 1][ObservedBoxY-1][ObservedBoxX])
            occupancyGrid[ObservedBoxT - 1][ObservedBoxY-1][ObservedBoxX] = temp * (1.25 / 100)
        if (plusT):
            temp = 1 if (occupancyGrid[ObservedBoxT + 1][ObservedBoxY-1][ObservedBoxX] == 0) else (
                occupancyGrid[ObservedBoxT + 1][ObservedBoxY-1][ObservedBoxX])
            occupancyGrid[ObservedBoxT + 1][ObservedBoxY-1][ObservedBoxX] = temp * (1.25 / 100)

        if (plusX):
            temp = 1 if (occupancyGrid[ObservedBoxT][ObservedBoxY-1][ObservedBoxX + 1]== 0) else(
                occupancyGrid[ObservedBoxT][ObservedBoxY-1][ObservedBoxX + 1])
            occupancyGrid[ObservedBoxT][ObservedBoxY-1][ObservedBoxX + 1] = temp * (2.5 / 100)
            if (minusT):
                temp = 1 if (occupancyGrid[ObservedBoxT - 1][ObservedBoxY-1][ObservedBoxX + 1] == 0) else (
                    occupancyGrid[ObservedBoxT - 1][ObservedBoxY-1][ObservedBoxX + 1])
                occupancyGrid[ObservedBoxT - 1][ObservedBoxY-1][ObservedBoxX + 1] = temp * (1.25 / 100)
            if (plusT):
                temp = 1 if (occupancyGrid[ObservedBoxT + 1][ObservedBoxY-1][ObservedBoxX + 1] == 0) else (
                    occupancyGrid[ObservedBoxT + 1][ObservedBoxY-1][ObservedBoxX + 1])
                occupancyGrid[ObservedBoxT + 1][ObservedBoxY-1][ObservedBoxX + 1] = temp * (1.25 / 100)

        if (minusX):
            temp = 1 if (occupancyGrid[ObservedBoxT][ObservedBoxY-1][ObservedBoxX - 1]== 0) else(
                occupancyGrid[ObservedBoxT][ObservedBoxY-1][ObservedBoxX - 1])
            occupancyGrid[ObservedBoxT][ObservedBoxY-1][ObservedBoxX - 1] = temp * (2.5 / 100)
            if (minusT):
                temp = 1 if (occupancyGrid[ObservedBoxT - 1][ObservedBoxY-1][ObservedBoxX - 1] == 0) else (
                    occupancyGrid[ObservedBoxT - 1][ObservedBoxY-1][ObservedBoxX - 1])
                occupancyGrid[ObservedBoxT - 1][ObservedBoxY-1][ObservedBoxX - 1] = temp * (1.25 / 100)
            if (plusT):
                temp = 1 if (occupancyGrid[ObservedBoxT + 1][ObservedBoxY-1][ObservedBoxX - 1] == 0) else (
                    occupancyGrid[ObservedBoxT + 1][ObservedBoxY-1][ObservedBoxX - 1])
                occupancyGrid[ObservedBoxT + 1][ObservedBoxY-1][ObservedBoxX - 1] = temp * (1.25 / 100)

    if (plusY):
        temp = 1 if (occupancyGrid[ObservedBoxT][ObservedBoxY + 1][ObservedBoxX]== 0) else(
            occupancyGrid[ObservedBoxT][ObservedBoxY + 1][ObservedBoxX])
        occupancyGrid[ObservedBoxT][ObservedBoxY + 1][ObservedBoxX] = temp * (2.5 / 100)
        if (minusT):
            temp = 1 if (occupancyGrid[ObservedBoxT - 1][ObservedBoxY + 1][ObservedBoxX] == 0) else (
            occupancyGrid[ObservedBoxT - 1][ObservedBoxY + 1][ObservedBoxX])
            occupancyGrid[ObservedBoxT - 1][ObservedBoxY + 1][ObservedBoxX] = temp * (1.25 / 100)
        if (plusT):
            temp = 1 if (occupancyGrid[ObservedBoxT + 1][ObservedBoxY + 1][ObservedBoxX] == 0) else (
            occupancyGrid[ObservedBoxT + 1][ObservedBoxY + 1][ObservedBoxX])
            occupancyGrid[ObservedBoxT + 1][ObservedBoxY + 1][ObservedBoxX] = temp * (1.25 / 100)

        if (plusX):
            temp = 1 if (occupancyGrid[ObservedBoxT][ObservedBoxY+1][ObservedBoxX + 1]== 0) else(
                occupancyGrid[ObservedBoxT][ObservedBoxY+1][ObservedBoxX + 1])
            occupancyGrid[ObservedBoxT][ObservedBoxY+1][ObservedBoxX + 1] = temp * (2.5 / 100)
            if (minusT):
                temp = 1 if (occupancyGrid[ObservedBoxT - 1][ObservedBoxY+1][ObservedBoxX + 1] == 0) else (
                    occupancyGrid[ObservedBoxT - 1][ObservedBoxY+1][ObservedBoxX + 1])
                occupancyGrid[ObservedBoxT - 1][ObservedBoxY+1][ObservedBoxX + 1] = temp * (1.25 / 100)
            if (plusT):
                temp = 1 if (occupancyGrid[ObservedBoxT + 1][ObservedBoxY+1][ObservedBoxX + 1] == 0) else (
                    occupancyGrid[ObservedBoxT + 1][ObservedBoxY+1][ObservedBoxX + 1])
                occupancyGrid[ObservedBoxT + 1][ObservedBoxY+1][ObservedBoxX + 1] = temp * (1.25 / 100)

        if (minusX):
            temp = 1 if (occupancyGrid[ObservedBoxT][ObservedBoxY+1][ObservedBoxX - 1]== 0) else(
                occupancyGrid[ObservedBoxT][ObservedBoxY+1][ObservedBoxX - 1])
            occupancyGrid[ObservedBoxT][ObservedBoxY+1][ObservedBoxX - 1] = temp * (2.5 / 100)
            if (minusT):
                temp = 1 if (occupancyGrid[ObservedBoxT - 1][ObservedBoxY+1][ObservedBoxX - 1] == 0) else (
                    occupancyGrid[ObservedBoxT - 1][ObservedBoxY+1][ObservedBoxX - 1])
                occupancyGrid[ObservedBoxT - 1][ObservedBoxY+1][ObservedBoxX - 1] = temp * (1.25 / 100)
            if (plusT):
                temp = 1 if (occupancyGrid[ObservedBoxT + 1][ObservedBoxY+1][ObservedBoxX - 1] == 0) else (
                    occupancyGrid[ObservedBoxT + 1][ObservedBoxY+1][ObservedBoxX - 1])
                occupancyGrid[ObservedBoxT + 1][ObservedBoxY+1][ObservedBoxX - 1] = temp * (1.25 / 100)


def achiever(bagPath):
    rospy.init_node('achiever', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    global occupancyGrid
    distributeProbabilities()


    while not rospy.is_shutdown():
        #bag = rosbag.Bag('/home/first/catkin_ws/src/lab4/scripts/grid.bag')
        #bag = rosbag.Bag('grid.bag')
        bag = rosbag.Bag(bagPath)
        marker = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        markers = MarkerArray()
        tag0 = Point()
        tag0.x = tag0x
        tag0.y = tag0y
        tag0.z = 0
        tag1 = Point()
        tag1.x = tag1x
        tag1.y = tag1y
        tag1.z = 0
        tag2 = Point()
        tag2.x = tag2x
        tag2.y = tag2y
        tag2.z = 0
        tag3 = Point()
        tag3.x = tag3x
        tag3.y = tag3y
        tag3.z = 0
        tag4 = Point()
        tag4.x = tag4x
        tag4.y = tag4y
        tag4.z = 0
        tag5 = Point()
        tag5.x = tag5x
        tag5.y = tag5y
        tag5.z = 0
        points = Marker()
        points.id = 0
        points.type = Marker.POINTS
        points.ns = 'gridTags'
        points.scale.x = 0.2;
        points.scale.y = 0.2;
        points.header.frame_id = 'base_laser_link'
        points.color.g = 1.0
        points.color.a = 1.0
        points.points.append(tag0)
        points.points.append(tag1)
        points.points.append(tag2)
        points.points.append(tag3)
        points.points.append(tag4)
        points.points.append(tag5)
        markers.markers.append(points)
        finalPointX = 0.0
        finalPointY = 0.0
        for topic, msg, t in bag.read_messages(topics=['Movements', 'Observations']):
            if(topic == 'Movements'):
                motionDetected(msg)
            elif(topic == 'Observations'):
                observationDetected(msg)

            myPosition = Marker()
            myPosition.id = 1
            myPosition.type = Marker.LINE_STRIP
            myPosition.ns = 'gridTags'
            myPosition.scale.x = 0.1;
            myPosition.scale.y = 0.1;
            myPosition.header.frame_id = 'base_laser_link'
            for i in range(len(lx)):
                myPos = Point()
                myPos.x = ((lx[i] * 0.2) + 0.1)#lx[i]
                myPos.y = ((ly[i] * 0.2) + 0.1)#ly[i]
                myPos.z = 0#lt[i]
                myPosition.color.g = 1.0
                myPosition.color.a = 1.0
                myPosition.color.b = 1.0
                myPosition.color.r = 1.0
                myPosition.points.append(myPos)
                if(i == len(lx)-1):
                    finalPointX = ((lx[i] * 0.2) + 0.1)
                    finalPointY = ((ly[i] * 0.2) + 0.1)

            markers.markers.append(myPosition)
            marker.publish(markers)

        rospy.logerr("Final Point- x:" + str(finalPointX) + " y:" + str(finalPointY))
        bag.close()
        rate.sleep()

def motionDetected(msg):
    motionMsg = Motion()
    motionMsg.timeTag = msg.timeTag
    motionMsg.rotation1 = msg.rotation1
    motionMsg.translation = msg.translation
    motionMsg.rotation2 = msg.rotation2
    eulerRotation1 = tf.transformations.euler_from_quaternion([motionMsg.rotation1.x, motionMsg.rotation1.y, motionMsg.rotation1.z, motionMsg.rotation1.w])
    yaw1 = eulerRotation1[2]
    eulerRotation2 = tf.transformations.euler_from_quaternion([motionMsg.rotation2.x, motionMsg.rotation2.y, motionMsg.rotation2.z, motionMsg.rotation2.w])
    yaw2 = eulerRotation2[2]
    #rospy.logerr("motion: " + "yaw1: "+ str(yaw1) + " trans: " +str(msg.translation) + " yaw2: " + str(yaw2)+ " time: "+ str(msg.timeTag))
    global occupancyGrid, ObservedPoseX, ObservedPoseY, ObservedPoseT, ObservedBoxX, ObservedBoxY, ObservedBoxT
    for t in range(len(occupancyGrid)):
        for y in range(len(occupancyGrid[t])):
            for x in range(len(occupancyGrid[y])):
                if(occupancyGrid[t][y][x]!=0):
                    currentPoseX = ((x*0.2)+0.1)
                    currentPoseY = ((y*0.2)+0.1)
                    currentPoseT = ((t*10) + 0.1)
                    ObservedPoseX = currentPoseX - (motionMsg.translation * (math.cos(math.radians(currentPoseT)+yaw1)))
                    ObservedPoseY = currentPoseY - (motionMsg.translation * (math.sin(math.radians(currentPoseT)+yaw1)))
                    ObservedPoseT = currentPoseT+math.degrees(yaw1)+math.degrees(yaw2)
                    ObservedBoxX = int(math.ceil(ObservedPoseX/0.2))
                    ObservedBoxY = int(math.ceil(ObservedPoseY/0.2))
                    ObservedBoxT = int(math.ceil(ObservedPoseT/10))
                    if(ObservedBoxX<=34 & ObservedBoxX>=0) &(ObservedBoxY<=34 & ObservedBoxY>=0) &(ObservedBoxT<=35 & ObservedBoxT>=0):
                        distributeProbabilities()

def observationDetected(msg):
    ObservationMsg = Observation()
    ObservationMsg.timeTag = msg.timeTag
    ObservationMsg.tagNum = msg.tagNum
    ObservationMsg.range = msg.range
    ObservationMsg.bearing = msg.bearing
    (tagX, tagY) = tags.get(ObservationMsg.tagNum, (0.0, 0,0))
    eulerBearing = tf.transformations.euler_from_quaternion([ObservationMsg.bearing.x, ObservationMsg.bearing.y, ObservationMsg.bearing.z, ObservationMsg.bearing.w])
    yaw = eulerBearing[2]
    #rospy.logerr("observation: " + "tag: " +str(msg.tagNum) + " range: " +str(msg.range) + " yaw: " + str(yaw)+ " time: "+ str(msg.timeTag))
    global lt, lx, ly, occupancyGrid, ObservedPoseX, ObservedPoseY, ObservedPoseT, ObservedBoxX, ObservedBoxY, ObservedBoxT

    minDist = 9000
    minDistT = 0
    minDistX = 0
    minDistY = 0
    maxProbability = 0
    maxProbT = 0
    maxProbX = 0
    maxProbY = 0

    for t in range(len(occupancyGrid)):
        for y in range(len(occupancyGrid[t])):
            for x in range(len(occupancyGrid[y])):
                if (occupancyGrid[t][y][x] != 0):
                    if(occupancyGrid[t][y][x]>maxProbability):
                        maxProbability = occupancyGrid[t][y][x]
                        maxProbT = t
                        maxProbX = x
                        maxProbY = y
                    currentPoseX = ((x * 0.2) + 0.1)
                    currentPoseY = ((y * 0.2) + 0.1)
                    currentPoseT = ((t * 10) + 0.1)
                    calTagX = currentPoseX + (ObservationMsg.range * (math.cos(yaw+math.radians(currentPoseT))))
                    calTagY = currentPoseY + (ObservationMsg.range * (math.sin(yaw+math.radians(currentPoseT))))
                    dist = math.hypot(tagX - calTagX, tagY - calTagY)
                    if(dist<minDist):
                        minDist = dist
                        minDistT = t
                        minDistX = x
                        minDistY = y
    lt.append(maxProbT)
    lx.append(maxProbX)
    ly.append(maxProbY)
    ObservedBoxX = minDistX
    ObservedBoxY = minDistY
    ObservedBoxT = minDistT
    ObservedPoseX = ((x * 0.2) + 0.1)
    ObservedPoseY = ((y * 0.2) + 0.1)
    ObservedPoseT = ((t * 10) + 0.1)
    occupancyGrid = [[[0 for x in range(35)] for y in range(35)] for t in range(36)]
    distributeProbabilities()

if __name__ == '__main__':
	    try:
		achiever(sys.argv[1])
	    except rospy.ROSInterruptException:
		pass
