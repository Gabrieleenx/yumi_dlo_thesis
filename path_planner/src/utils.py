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

