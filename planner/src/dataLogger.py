#!/usr/bin/env python3

import numpy as np
import tf
import utils
import rospy
import pickle

class DataWrapper(object):
    def __init__(self, time, data):
        self.time = time
        self.data = data

class Logger(object):
    def __init__(self):
        self.gripperPose = [] #
        self.DLOPoints = [] #
        self.pathplannerState = [] #
        self.solutionIndividual = [] #
        self.controllerState = [] #
        self.currentTask = [] #
        self.msgSent = [] #
        self.jointPosition = [] #
        self.fixturesObj = [] #

    def appendGripperPose(self, data):
        obj = DataWrapper(time=rospy.Time.now().to_sec(),\
                        data=data)
        self.gripperPose.append(obj)

    def appendDLOPoints(self, data):
        obj = DataWrapper(time=rospy.Time.now().to_sec(),\
                        data=data)
        self.DLOPoints.append(obj)

    def appendPathplannerState(self, data):
        obj = DataWrapper(time=rospy.Time.now().to_sec(),\
                        data=data)
        self.pathplannerState.append(obj)

    def appendControllerState(self, data):
        obj = DataWrapper(time=rospy.Time.now().to_sec(),\
                        data=data)
        self.controllerState.append(obj)

    def appendMsgSent(self, data):
        obj = DataWrapper(time=rospy.Time.now().to_sec(),\
                        data=data)
        self.msgSent.append(obj)

    def appendJointPosition(self, data):
        obj = DataWrapper(time=rospy.Time.now().to_sec(),\
                        data=data)
        self.jointPosition.append(obj)

    def appendFixturesObj(self, data):
        obj = DataWrapper(time=rospy.Time.now().to_sec(),\
                        data=data)
        self.fixturesObj.append(obj)
    
    def appendSolutionIndividual(self, data):
        obj = DataWrapper(time=rospy.Time.now().to_sec(),\
                        data=data)
        self.solutionIndividual.append(obj)

    def appendCurrentTask(self, data):
        obj = DataWrapper(time=rospy.Time.now().to_sec(),\
                        data=data)
        self.currentTask.append(obj)

def saveObjectPickle(fileName, object_):
    print('Saving data')
    file_save = open(fileName, 'wb')
    pickle.dump(object_, file_save, protocol=pickle.HIGHEST_PROTOCOL)
    file_save.close()