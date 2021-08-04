#!/usr/bin/env python3

import numpy as np
import tf
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation


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
        for ii in range(self.dynamicObjects[i].lines.shape[0]):
            lineIndex = self.dynamicObjects[i].lines[ii]
            lineVerticeis = self.dynamicObjects[i].verticesTransformed[lineIndex]
            normAxis = self.calcNormalAxisFromLine(lineVerticeis)
            aMax, aMin, bMax, bMin = self.calcMinMaxProjectionOnAxis(i, j, normAxis)
            if not self.calcOverlap(aMax, aMin, bMax, bMin):
                return True
        
        for ii in range(self.objects[j].lines.shape[0]):
            lineIndex = self.objects[j].lines[ii]
            lineVerticeis = self.objects[j].verticesTransformed[lineIndex]
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

def main():
    global position, orientation, o2, obj2, obj1
    rospy.init_node('collisionTest', anonymous=True) 
    vertices = np.array([[0.05, 0.05],[0.05, -0.05],[-0.05, -0.05],[-0.05, 0.05]])
    lines = np.array([[0,1],[1,2],[2,3],[3,0]])
    height = np.array([0,0.1])
    obj1 = CollisionObject(vertices, lines, height)
    obj2 = CollisionObject(vertices, lines, height)

    collisionCheck = CollisionCheck()
    collisionCheck.updateStaticObjects([obj1])

    position = np.array([0.4,0,0])
    orientation = np.array([0,0,0,1])
    o2 = 0
    obj2.updatePose(position, orientation)
    collisionCheck.updateDynamicObjects([obj2])
    #for i in range(1000):
    #    collisionCheck.checkCollision()


    fig, ax = plt.subplots()

    line1, = ax.plot([], [], 'o-', lw=2)
    line2, = ax.plot([], [], 'o-', lw=2)
    ax.set_xlim(-0.5, 0.5)
    ax.set_ylim(-0.5, 0.5)

    def animate(i):
        global position, orientation, o2, obj2, obj1
        x_data = []
        y_data = []

        for ii in range(obj1.lines.shape[0]):
            lineIndex = obj1.lines[ii]
            lineVerticeis = obj1.verticesTransformed[lineIndex]
            x_data.extend(lineVerticeis[:,0].tolist())
            y_data.extend(lineVerticeis[:,1].tolist())

        line1.set_data(x_data,y_data)
        x_data = []
        y_data = []
        #line1.set_color('r')
        position = position - np.array([0.0005,0,0])
        o2 += 0.01
        if position[0] < 0:
            position = np.array([0.4,0,0])
            orientation = np.array([0,0,0,1])
            o2 = 0
            vertices = np.array([[0.05, 0.05], [0.0707, -0.0], [0.05, -0.05], [0.0, -0.0707],[-0.05, -0.05], [-0.0707, 0.0],[-0.05, 0.05], [0.0, 0.0707]])
            lines = np.array([[0,1],[1,2],[2,3],[3,4],[4,5],[5,6],[6,7],[7,0]])
            height = np.array([0,0.1])
            obj2 = CollisionObject(vertices, lines, height)
            vertices = np.array([[0.05, 0.05],[0.05, -0.05],[-0.05, -0.05]])
            lines = np.array([[0,1],[1,2],[2,0]])
            height = np.array([0,0.1])
            obj1 = CollisionObject(vertices, lines, height)
            orientation1 = tf.transformations.quaternion_from_euler(np.pi/1.1, 0, 0, 'rzyx')
            obj1.updatePose(np.zeros(3), orientation1)
        orientation = tf.transformations.quaternion_from_euler(o2, 0, 0, 'rzyx')

        obj2.updatePose(position, orientation)
        collisionCheck.updateDynamicObjects([obj2])
        for ii in range(obj2.lines.shape[0]):
            lineIndex = obj2.lines[ii]
            lineVerticeis = obj2.verticesTransformed[lineIndex]
            x_data.extend(lineVerticeis[:,0].tolist())
            y_data.extend(lineVerticeis[:,1].tolist())

        line2.set_data(x_data,y_data)

        if collisionCheck.checkCollision():
            line1.set_color('r')
            line2.set_color('r')
        else:
            line1.set_color('b')
            line2.set_color('b')
        return line1, line2

    ani = animation.FuncAnimation(
        fig, animate, interval=10, blit=True, save_count=50)


    plt.show()

if __name__ == '__main__':
    main()
