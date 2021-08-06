#!/usr/bin/env python3

import numpy as np
import tf
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class CollisionObject(object):
    def __init__(self, vertices, lines, height):
        self.position = np.zeros(3)
        self.orientation = np.array([0,0,0,1])
        self.vertices = vertices
        self.verticesHomogenious = self.extendVertices(vertices) # np.array([[x,y],[x,y],[x,y]])
        self.lines = lines # np.array([[1,2],[0,1],[2,3]]), pair vertex indecies to create line
        self.height = height
        self.heightTransformed = height
        self.verticesTransformed = vertices # np.array([[x,y],[x,y],[x,y]]), relative position to pose. 
        self.maxDist = self.calcMaxDist()
        self.transformer = tf.TransformerROS(True, rospy.Duration(0.01))

    def calcMaxDist(self):
        dist = np.zeros(self.vertices.shape[0])
        for i in range(self.vertices.shape[0]):
            dist[i] = np.linalg.norm(self.vertices[i])
        return dist.max()

    def transformVertices(self):
        tfMatrixRight = self.transformer.fromTranslationRotation(translation=self.position, rotation=self.orientation)
        self.verticesTransformed = tfMatrixRight.dot(self.verticesHomogenious.T).T[:,0:2]

    def extendVertices(self, vertices):
        extendedVertices = np.hstack([vertices, np.zeros((vertices.shape[0], 1)), np.ones((vertices.shape[0], 1))])
        return extendedVertices

    def transformHeight(self):
        self.heightTransformed = self.height + self.position[2]

    def updatePose(self, position, orientation):
        self.position = position 
        self.orientation = orientation
        self.transformHeight()
        self.transformVertices()




class CollisionCheck(object):
    def __init__(self):
        self.staticObjects = [] # static objcets not checked agains each other 
        self.dynamicObjects = []
        self.objects = self.dynamicObjects + self.staticObjects  # concatinate lists
        self.linePub = rospy.Publisher('/lineViz', Marker, queue_size=1)

    def checkCollision(self):
        collision = False

        for i in range(len(self.dynamicObjects)):
            for j in range(len(self.objects)):
                if i == j:
                    continue

                if self.checkMaxDist(i, j):
                    continue

                if self.checkHeight(i, j):
                    continue
                if self.findSeperatingAxis(i,j):
                    continue
                else:
                    collision = True
                    return collision
    
        return collision 
                    
    def updateStaticObjects(self, staticObjects):
        self.staticObjects = staticObjects
        self.objects = self.dynamicObjects + self.staticObjects  # concatinate lists

    def updateDynamicObjects(self, dynamicObjects):
        self.dynamicObjects = dynamicObjects
        self.objects = self.dynamicObjects + self.staticObjects  # concatinate lists

    def findSeperatingAxis(self, i, j):

        for ii in range(self.objects[j].lines.shape[0]):
            lineIndex = self.objects[j].lines[ii]
            lineVerticeis = self.objects[j].verticesTransformed[lineIndex]
            normAxis = self.calcNormalAxisFromLine(lineVerticeis)
            aMax, aMin, bMax, bMin = self.calcMinMaxProjectionOnAxis(i, j, normAxis)
            if not self.calcOverlap(aMax, aMin, bMax, bMin):
                return True

        for ii in range(self.dynamicObjects[i].lines.shape[0]):
            lineIndex = self.dynamicObjects[i].lines[ii]
            lineVerticeis = self.dynamicObjects[i].verticesTransformed[lineIndex]
            normAxis = self.calcNormalAxisFromLine(lineVerticeis)
            aMax, aMin, bMax, bMin = self.calcMinMaxProjectionOnAxis(i, j, normAxis)
            if not self.calcOverlap(aMax, aMin, bMax, bMin):
                return True

        return False

    def calcNormalAxisFromLine(self, lineVerticeis):
        axis = lineVerticeis[1] - lineVerticeis[0]
        normal = np.array([axis[1], -axis[0]])
        return normal

    def calcMinMaxProjectionOnAxis(self, i, j, normAxis):
        projectionsA = self.dynamicObjects[i].verticesTransformed.dot(normAxis)
        aMax = np.max(projectionsA)
        aMin = np.min(projectionsA)
        projectionsB = self.objects[j].verticesTransformed.dot(normAxis)
        bMax = np.max(projectionsB)
        bMin = np.min(projectionsB)
        return aMax, aMin, bMax, bMin        

    def calcOverlap(self, aMax, aMin, bMax, bMin):
        return aMin<=bMax<=aMax or aMin<=bMin<=aMax or bMin<=aMax<=bMax or bMin<=aMin<=bMax

    def checkMaxDist(self, i, j):
        distMin = self.dynamicObjects[i].maxDist + self.objects[j].maxDist
        diff = self.dynamicObjects[i].position[0:2] - self.objects[i].position[0:2]
        dist = np.linalg.norm(diff)
        if dist > distMin:
            return True
        else:
            return False

    def checkHeight(self, i, j):
        aMax = self.dynamicObjects[i].heightTransformed[1]
        aMin = self.dynamicObjects[i].heightTransformed[0]
        bMax = self.objects[j].heightTransformed[1]
        bMin = self.objects[j].heightTransformed[0]
        return not self.calcOverlap(aMax, aMin, bMax, bMin)

    def plotPolygons(self):
        # https://answers.ros.org/question/203782/rviz-marker-line_strip-is-not-displayed/
        marker = Marker()
        marker.header.frame_id = "yumi_base_link"
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        
        # marker scale
        marker.scale.x = 0.002
        marker.scale.y = 0.002
        marker.scale.z = 0.002

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        objectList = self.objects

        # marker line points
        marker.points = []

        for j in range(len(objectList)):


            for ii in range(objectList[j].lines.shape[0]):
                lineIndex = objectList[j].lines[ii]
                lineVerticeis = objectList[j].verticesTransformed[lineIndex]
                height = objectList[j].heightTransformed


                x_data = lineVerticeis[:,0].tolist()
                y_data = lineVerticeis[:,1].tolist()

                point1 = Point()
                point1.x = x_data[0]
                point1.y = y_data[0]
                point1.z = height[0]
                marker.points.append(point1)

                point2 = Point()
                point2.x = x_data[1]
                point2.y = y_data[1]
                point2.z = height[0]
                marker.points.append(point2)

                point3 = Point()
                point3.x = x_data[0]
                point3.y = y_data[0]
                point3.z = height[1]
                marker.points.append(point1)
                marker.points.append(point3)

                point4 = Point()
                point4.x = x_data[1]
                point4.y = y_data[1]
                point4.z = height[1]
                marker.points.append(point2)
                marker.points.append(point4)


                point5 = Point()
                point5.x = x_data[0]
                point5.y = y_data[0]
                point5.z = height[1]
                marker.points.append(point5)

                point6 = Point()
                point6.x = x_data[1]
                point6.y = y_data[1]
                point6.z = height[1]
                marker.points.append(point6)
        self.linePub.publish(marker)

def setupStaticObjects(listOfObjects):
    listOfCollisionObjects = []

    # collision object for the fixtures. 
    verticesBase = np.array([[-0.016, 0.03],[0.064, 0.03],[0.064, -0.03],[-0.016, -0.03]])
    linesBase = np.array([[0,1],[1,2],[2,3],[3,0]])
    heightBase = np.array([0,0.02])

    verticesClip = np.array([[-0.015, 0.01],[0.015, 0.01],[0.015, -0.01],[-0.015, -0.01]])
    linesClip = np.array([[0,1],[1,2],[2,3],[3,0]])
    heightClip = np.array([0,0.07])

    for i in range(len(listOfObjects)):
        position = listOfObjects[i].getBasePosition()
        orientation = listOfObjects[i].getOrientation()
        
        objFixtureBase = CollisionObject(verticesBase, linesBase, heightBase)
        objFixtureBase.updatePose(position, orientation) 

        objFixtureClip = CollisionObject(verticesClip, linesClip, heightClip)
        objFixtureClip.updatePose(position, orientation)
        
        listOfCollisionObjects.extend([objFixtureBase, objFixtureClip])
    
    # collision objects for yumi 
    
    # yumi stand
    verticesYuMiStand = np.array([[-0.0675, 0.05],[0.0675, 0.05],[0.0675, -0.05],[-0.0675, -0.05]])
    linesYuMiStand = np.array([[0,1],[1,2],[2,3],[3,0]])
    heightYuMiStand = np.array([0,0.07])
    
    objYumi = CollisionObject(verticesYuMiStand, linesYuMiStand, heightYuMiStand)
    objYumi.updatePose(position=np.array([0.0675, 0.14, 0]), orientation=np.array([0,0,0,1]))
    listOfCollisionObjects.extend([objYumi])

    objYumi = CollisionObject(verticesYuMiStand, linesYuMiStand, heightYuMiStand)
    objYumi.updatePose(position=np.array([0.0675, -0.14, 0]), orientation=np.array([0,0,0,1]))
    listOfCollisionObjects.extend([objYumi])

    # yumi base upper
    verticesYuMiUpperBase = np.array([[-0.05, 0.14],[0.05, 0.14],[0.05, -0.14],[-0.05, -0.14]])
    linesYuMiUpperBase = np.array([[0,1],[1,2],[2,3],[3,0]])
    heightYuMiUpperBase = np.array([-0.075,0.075])
    objYumi = CollisionObject(verticesYuMiUpperBase, linesYuMiUpperBase, heightYuMiUpperBase)
    objYumi.updatePose(position=np.array([0.02, 0, 0.39]), orientation=np.array([0,0,0,1]))
    listOfCollisionObjects.extend([objYumi])
    
    # yumi shoulders 
    verticesYuMiUpperBase = np.array([[-0.05, 0.22],[0.05, 0.22],[0.05, -0.22],[-0.05, -0.22]])
    linesYuMiUpperBase = np.array([[0,1],[1,2],[2,3],[3,0]])
    heightYuMiUpperBase = np.array([-0.075,0.075])
    objYumi = CollisionObject(verticesYuMiUpperBase, linesYuMiUpperBase, heightYuMiUpperBase)
    objYumi.updatePose(position=np.array([0.12, 0, 0.47]), orientation=np.array([0,0,0,1]))
    listOfCollisionObjects.extend([objYumi])

    # limit workspace    
    verticesSpace = np.array([[0, -0.6], [0, 0.6]])
    linesSpace = np.array([[0,1]])
    heightSpace = np.array([0, 0.5])
    objSpace = CollisionObject(verticesSpace, linesSpace, heightSpace)
    objSpace.updatePose(position=np.array([0, 0, 0]), orientation=np.array([0,0,0,1]))
    listOfCollisionObjects.extend([objSpace])
    return listOfCollisionObjects



def calcGripperObjects():
    v = 0
    verticesW = []
    for i in range(8):
        x = np.cos(v) * 0.06
        y = np.sin(v) * 0.06
        v += 2* np.pi/8
        verticesW.extend([[x, y]])
    verticesW = np.array(verticesW)
    linesW = np.array([[0,1],[1,2],[2,3],[3,4],[4,5],[5,6],[6,7],[7,0]])
    heightW = np.array([0.14, 0.22])
    verticesH =  np.array([[-0.0425, 0.0325],[0.0425, 0.0325],[0.0425, -0.0325],[-0.0425, -0.0325]])
    linesH = np.array([[0,1],[1,2],[2,3],[3,0]])
    heightH = np.array([0.05, 0.12])
    verticesF =  np.array([[-0.032, 0.0125],[0.032, 0.0125],[0.032, -0.0125],[-0.032, -0.0125]])
    linesF = np.array([[0,1],[1,2],[2,3],[3,0]])
    heightF = np.array([0, 0.04])

    gripperRightWrist = CollisionObject(verticesW, linesW, heightW)
    gripperRightHand = CollisionObject(verticesH, linesH, heightH)
    gripperRightFigers = CollisionObject(verticesF, linesF, heightF)

    gripperLeftWrist = CollisionObject(verticesW, linesW, heightW)
    gripperLeftHand = CollisionObject(verticesH, linesH, heightH)
    gripperLeftFigers = CollisionObject(verticesF, linesF, heightF)
    
    objectsR = [gripperRightFigers, gripperRightHand, gripperRightWrist]
    objectsL = [gripperLeftFigers, gripperLeftHand, gripperLeftWrist]

    return objectsR, objectsL

