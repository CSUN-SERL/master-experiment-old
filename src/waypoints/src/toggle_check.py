#!/usr/bin/env python

from socketIO_client import BaseNamespace
from socketIO_client import LoggingNamespace
from socketIO_client import SocketIO

import rospy
import roslib
import time

from std_msgs.msg import String
from threading import Thread

#initial position of the robot

toggle = True
# INPUT_URI='http://ec2-52-24-126-225.us-west-2.compute.amazonaws.com'
INPUT_URI='http://192.168.1.11'
# INPUT_PORT=81
INPUT_PORT=8000
pub = None

class Namespace(BaseNamespace):
    
    def tester_publish(self):
        global toggle
        global pub
        rospy.loginfo('Set toggle to {}'.format(str(toggle)))
        pub.publish(str(toggle))

    def on_cmtogglemanualcontrol(self, *args):
        '''
        If a message is sent on the cm-toggle-manual-control topic then the publisher
        publishes the same message via ros to a topic which each robot will listen too.
        The data in this topic will only contain a number. This number will specify 
        which robot will stop it's 'autonomous' movement
        '''
        global pub
        rospy.loginfo('Message recieved: {}'.format(str(args[0])))
        rospy.loginfo('Toggled robot{}\'s autonomous control'.format(str(args[0])))
        print(args[0])
        pub.publish(str(args[0]))
    '''
    def on_tester(self, *args):
        global toggle
        previous_toggle = toggle
        toggle = not toggle
        self.tester_publish()
    '''

def init_publisher():
    global pub
    pub = rospy.Publisher('toggle', String, queue_size=1)
    rospy.init_node('toggle_check', anonymous=True)


def main():
    init_publisher()

    socketIO = SocketIO(INPUT_URI, INPUT_PORT)
    socket_namespace = socketIO.define(Namespace, '/socket.io')

    rospy.loginfo('Started toggle_checker listening on uri: {}'.format(INPUT_URI))
    while True:
        socketIO.wait(seconds=1)


if __name__ == "__main__":
    main()
