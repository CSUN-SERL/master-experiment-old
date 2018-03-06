#!/usr/bin/env python
import rospy
import roslib
import actionlib
import time
import numpy as np
from move_base_msgs.msg import *
from actionlib import SimpleActionClient, GoalStatus
from geometry_msgs.msg import *
f = open('/home/serl/Documents/multi_bots_v3/src/waypoints/src/points.txt' , 'w')
count = 0


def callback(data):
    global count
    global f
  
    x = data.pose.position.x
    y = data.pose.position.y
    w = data.pose.orientation.w
    z = data.pose.orientation.z

    if count == 0:
        f.write('[')   
    f.write('[')
    f.write(str(x))
    f.write(',')
    f.write(str(y))
    f.write(',')
    f.write(str(w))
    f.write(',')
    f.write(str(z))
    f.write(']')
    count = count + 1
    if count % 3 == 0:
        f.write('],[')
    else:
        f.write(',')
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
     
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/robot1/move_base/current_goal', PoseStamped, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    f.close()
if __name__ == '__main__':
    listener()
