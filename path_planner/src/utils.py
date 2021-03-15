#!/usr/bin/env python3

import numpy as np


class FramePose(object):
    def __init__(self):
        self.position = np.zeros(3)
        self.quaternion = np.zeros(4)

    def getQuaternion(self):
        return self.quaternion

    def getPosition(self):
        return self.position

    def update(self, position, orientation):
        self.position = np.asarray(position)
        self.quaternion = np.asarray(orientation)


class DLO(object):
    def __init__(self):
        self.points = np.zeros((50,3))
        self.lengthList = np.zeros(50)
        self.totalLenght = 0 

    def update(self, points):
        numPoints = np.shape(points)[0]
        self.points = np.copy(points)

        for i in range(numPoints-1):
            diff = points[i+1] - points[i]
            dist = np.linalg.norm(diff)
            self.lengthList[i+1] = self.lengthList[i] + dist
        
        self.totalLenght = self.lengthList[-1]

    def getCoord(self, length):
        if length < 0 or length > self.totalLenght:
            return False

        index1 = np.argwhere(self.lengthList >= length)[0,0]
        index0 = index1 - 1
        if index0 < 0:
            index0 = 0

        diff = self.points[index1] - self.points[index0]

        point = self.points[index0] + normalize(diff) * (length - self.lengthList[index0])

        return point

    def getLength(self):
        return self.totalLenght


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm
