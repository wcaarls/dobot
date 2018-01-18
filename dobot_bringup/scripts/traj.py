#!/usr/bin/env python

import rospy
from dobot_msgs.srv import SetCPCmd, SetCPParams, SetQueuedCmdStopExec, SetQueuedCmdStartExec

def octagon(cpcmd):
    cpcmd(0, 10, 10, 0, 0)
    cpcmd(0, 10, 0, 0, 0)
    cpcmd(0, 10, -10, 0, 0)
    cpcmd(0, 0, -10, 0, 0)
    cpcmd(0, -10, -10, 0, 0)
    cpcmd(0, -10, 0, 0, 0)
    cpcmd(0, -10, 10, 0, 0)
    cpcmd(0, 0, 10, 0, 0)
    
def repeat(cpcmd, cppar):
    cpcmd(0, 1, 0, 0, 200)
    cpcmd(0, 25, 0, 0, 200)
    cpcmd(0, 25, 0, 0, 200)
    cppar(100, 100, 100, 0, 1)
    cpcmd(0, 25, 0, 0, 200)
    cpcmd(0, 25, 0, 0, 200)
    cpcmd(0, -25, 0, 0, 200)
    cpcmd(0, -25, 0, 0, 200)
    cppar(50, 50, 50, 0, 1)
    cpcmd(0, -25, 0, 0, 200)
    cpcmd(0, -25, 0, 0, 200)
    cpcmd(0, -1, 0, 0, 200)

def traj():
    rospy.init_node('traj')
    rospy.wait_for_service('/DobotServer/SetCPParams')
    cppar = rospy.ServiceProxy('/DobotServer/SetCPParams', SetCPParams)
    rospy.wait_for_service('/DobotServer/SetCPCmd')
    cpcmd = rospy.ServiceProxy('/DobotServer/SetCPCmd', SetCPCmd, persistent=True)
    rospy.wait_for_service('/DobotServer/SetQueuedCmdStopExec')
    cpstop = rospy.ServiceProxy('/DobotServer/SetQueuedCmdStopExec', SetQueuedCmdStopExec, persistent=True)
    rospy.wait_for_service('/DobotServer/SetQueuedCmdStartExec')
    cpstart = rospy.ServiceProxy('/DobotServer/SetQueuedCmdStartExec', SetQueuedCmdStartExec, persistent=True)
    
    # planacc, juncvel, acc, rt, queued
    cppar(50, 50, 50, 0, 0)
    
#    cpstop()
    repeat(cpcmd, cppar)
#    cpstart()

if __name__ == '__main__':
    try:
        traj()
    except rospy.ROSInterruptException:
        pass

