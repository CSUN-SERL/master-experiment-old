#!/usr/bin/env python2.7

import rospy
from time import sleep
import yaml
import random

from nav_msgs.msg import Odometry

# Load the location information

# def main(robotnum):
#     rospy.init_node('odom_emulator' + str(robotnum))
#     pubtopic = '/robot' + str(robotnum) + '/odometry/filtered'
#     print 'Publishing to ' + pubtopic
#     pub = rospy.Publisher(pubtopic, Odometry, queue_size = 1)

#     locs = []
#     if robotnum == 1:
#         locs.append((2,18))
#         locs.append((60,20))
#     elif robotnum == 2:
#         locs.append((101, 201))
#         locs.append((151, 205))
    
#     current_loc = 0
#     while current_loc < len(locs):
#         raw_input("Press enter to move Robot " + str(robotnum) + " to next trigger zone")
#         msg = Odometry()
#         msg.pose.pose.position.x = locs[current_loc][0]
#         msg.pose.pose.position.y = locs[current_loc][1]

#         pub.publish(msg)
#         print("Published message")

#         current_loc += 1

def main():
    # init ros stuff
    print "init node"
    rospy.init_node('odom_emulator')

    print "Create publishers"
    pubtopicOne = '/test/robot1/odometry/filtered'
    global pubOne
    pubOne = rospy.Publisher(pubtopicOne, Odometry, queue_size = 2)

    pubtopicTwo = '/test/robot2/odometry/filtered'
    global pubTwo
    pubTwo = rospy.Publisher(pubtopicTwo, Odometry, queue_size = 2)

    pubtopicThree = '/test/robot3/odometry/filtered'
    global pubThree
    pubThree = rospy.Publisher(pubtopicThree, Odometry, queue_size = 2)

    pubtopicFour = '/test/robot4/odometry/filtered'
    global pubFour
    pubFour = rospy.Publisher(pubtopicFour, Odometry, queue_size = 2)

    locationYaml = yaml.safe_load(open('location_test.yaml', 'r'))['locations'].values()
    queryOrder = random.sample(xrange(100), 100)
    orderIndex = 0

    print "Main activated, sleeping"
    sleep(10)
    print "Done sleeping 1, sending odom"
    while True:
        if orderIndex >= len(queryOrder):
            break
        print "Sending next odometry message"
        nextQuery = locationYaml[queryOrder[orderIndex]]
        msg = Odometry()
        msg.pose.pose.position.x = nextQuery['x'] + 1
        msg.pose.pose.position.y = nextQuery['y'] + 1
        robotId = (queryOrder[orderIndex] % 4) + 1
        print "sending to " + str(robotId)
        print "query " + str(nextQuery['query'])

        if robotId == 1:
            print "if 1"
            pubOne.publish(msg)
        elif robotId == 2:
            print "if 2"
            pubTwo.publish(msg)
        elif robotId == 3:
            print "if 3"
            pubThree.publish(msg)
        else:
            print "if 4"
            pubFour.publish(msg)
        
        orderIndex += 1
        print "Sleeping"
        sleep(10)