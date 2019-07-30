#! /usr/bin/env python

import os

import rospy
import rospkg
import actionlib

import file_server.msg

class SendFileServer(object):
    _feedback = file_server.msg.SendFilesFeedback()
    
    def __init__(self, rate):
        self._actionName = "file_transfer_from_ros"
        self._scriptName = "File Transfer From ROS"
        self._rate = rate
        
        rospy.init_node(self._actionName)
        
        self._simpleActionServer = actionlib.SimpleActionServer(self._actionName, file_server.msg.FileTransferAction, execute_cb = self.execute_cb, auto_start = False)
        self._simpleActionServer.start()
    
    def findFilesInDir(self, dir, isRecursive):
        files = []
        if isRecursive:
            for r, d, f in os.walk(dir):
                for file in f:
                    files.append(os.path.join(r, file))
        else:
            for file in os.listdir(dir):
                files.append(dir + "/" + file)
        return files
    
    def findFilesInDirWithExt(self, dir, ext, isRecursive):
        files = []
        if isRecursive:
            for r, d, f in os.walk(dir):
                for file in f:
                    if ext in file:
                        files.append(os.path.join(r, file))
        else:
            for file in os.listdir(dir):
                if file.endswith(ext):
                    files.append(dir + "/" + file)
        return files
    
    def execute_cb(self, goal):
        rate = rospy.Rate(self._rate)
        
        files = []
        
        # Find and collect files
        # Handle Single File
        
        # Handle Package
        if goal.type == goal.PACKAGE :
            pkgPath = rospkg.RosPack().get_path(goal.identifier)
            rospy.loginfo("%s: Package path: %s" % (self._scriptName, pkgPath))
            if len(goal.extensions) == 0:
                files.extend(self.findFilesInDir(pkgPath, True))
            else:
                for type in goal.extensions:
                    filesOfType = self.findFilesInDirWithExt(pkgPath, type, True)
                    rospy.loginfo("%s: found %i files of type %s in package %s" % (self._scriptName, len(filesOfType), type, goal.identifier))
                    files.extend(filesOfType)
        # Handle Recursive
        
        # Send files as feedback
        for i in range(len(files)):
            if self._simpleActionServer.is_preempt_requested():
                rospy.loginfo("%s: Action preempted." % (self._scriptName))
                self._simpleActionServer.set_preempted()
                return
            file = files[i]
            rospy.loginfo("%s: Sending %s" % (self._scriptName, file))
            self._feedback.count = len(files)
            self._feedback.number = i + 1
            self._feedback.path = file
            self._feedback.content = open(file, "rb").read()
            self._simpleActionServer.publish_feedback(self._feedback)
            rate.sleep()
        rospy.loginfo("%s: Succeeded" % (self._scriptName))
        self._simpleActionServer.set_succeeded()

if __name__ == "__main__":
    SendFileServer(5)
    try:
        rospy.spin()
    except(KeyboardInterrupt, SystemExit):
        rospy.loginfo("Stop server")
