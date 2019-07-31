#! /usr/bin/env python

import os

import rospy
import rospkg
import actionlib

import file_server.msg

class SendFileServer(object):
    feedback = file_server.msg.FileTransferFeedback()
    
    def __init__(self, rate):
        self.actionName = "file_transfer_from_ros"
        self.scriptName = "File Transfer From ROS"
        
        self.files = []
        
        rospy.init_node(self.actionName)
        
        self.rate = rospy.Rate(rate)
        
        self.simpleActionServer = actionlib.SimpleActionServer(self.actionName, file_server.msg.FileTransferAction, self.execute_cb, auto_start = False)
        self.simpleActionServer.start()
    
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
    
    def getDefaultResult(self):
        return self.actionServer.ActionResultType()
    
    def execute_cb(self, goal):
        # Find and collect files.
        # If requests resources are not found, reject goal
        # Handle Single File
        if goal.type == goal.SINGLE:
            pass
        # Handle Package
        if goal.type == goal.PACKAGE:
            try:
                pkgPath = rospkg.RosPack().get_path(goal.identifier)
                rospy.loginfo("%s: Package '%s' is located at: %s" %(self.scriptName, goal.identifier, pkgPath))
                if len(goal.extensions) == 0:
                    rospy.loginfo("%s: Getting all files under directory")
                    self.files.extend(self.findFilesInDir(pkgPath, True))
                else:
                    for type in goal.extensions:
                        filesWithExt = self.findFilesInDirWithExt(pkgPath, type, true)
                        rospy.loginfo("%s: Found %i files with extension %s in package %s" % (self.scriptName, len(filesOfType), type, goal.identifier))
                        self.files.extend(filesOfType)
            except rospkg.common.ResourceNotFound:
                rospy.logerr("%s: Package '%s' not found. Goal aborted." % (self.scriptName, goal.identifier))
                self.simpleActionServer.set_aborted(text = "Package '%s' not found." % (goal.identifier))
                return
        # Handle Recursive
        if goal.type == goal.RECURSIVE:
            pass
        # Send files as feedback
        for i in range(len(self.files)):
            if self.simpleActionServer.is_preempt_requested():
                rospy.loginfo("%s: Action preempted." % (self.scriptName))
                self.simpleActionServer.set_preempted()
                return
            file = self.files[i]
            rospy.loginfo("%s: Sending %s" % (self.scriptName, file))
            self.feedback.count = len(self.files)
            self.feedback.number = i + 1
            self.feedback.path = file
            self.feedback.content = open(file, "rb").read()
            self.simpleActionServer.publish_feedback(self.feedback)
            self.rate.sleep()
        rospy.loginfo("%s: Succeeded" % (self.scriptName))
        self.simpleActionServer.set_succeeded()
        
    
if __name__ == "__main__":
    SendFileServer(5)
    rospy.spin()
