#!/usr/bin/env python

import rospy
import yaml
import sys

from rospy import ROSException
from nav_msgs.msg import Odometry
import multiprocessing
from ctypes import c_bool
#from multiprocessing import Process
#from audio_trigger_node.msg import Trigger
from detection_msgs.msg import AudioDetection
import shout

# Topic used to send logging message
pubtopic = '/sarwai_detection/detection_audio'
pub = rospy.Publisher(pubtopic, AudioDetection, queue_size = 1000)

# found locations
activatedLocations = {}

#Set up icecast shout service
s = shout.Shout()
s.host = 'localhost'
# s.port = 8050
# s.password = 'SERLstream'
s.port = 8050
s.password = 'hackme'
s.mount = 'testmount'
s.format = 'mp3'
#s.protocol = 'http'
# s.audio_info = {
#   shout.SHOUT_AI_BITRATE : ,
#   shout.SHOUT_AI_SAMPLERATE : ,
#   shout.SHOUT_AI_CHANNELS : ,
#   shout.SHOUT_AI_QUALITY : 
# }

s.open()

# Define function to loop background file to icecast

## Should bg loop be moved to audio_streamer?

bg_playing = multiprocessing.Value(c_bool, True)

def bgLoop():
    bgfilename = 'audio/audioloop.mp3'
    bg = open(bgfilename, 'rb')

    nbuf = bg.read(4096)
    while True:
        if bg_playing.value:
            buf = nbuf
            nbuf = bg.read(4096)
            if len(buf) == 0:
                print("looping bg")
                bg.close()
                bg = open(bgfilename)
                continue
            s.send(buf)
            s.sync()
        else:
            continue

def main():
    #rospy.init_node('listen_for_pose',anonymous = True)

    global trigger_list
    trigger_list = {}

    global query_list
    query_list = {}

    # define the class for our trigger location data
    class trigger_info(yaml.YAMLObject):
        yaml_tag = u'!Location'
        def __init__(self,x,y,width,height,query):
            self.x = x
            self.y = y
            self.w = width
            self.h = height
            self.q = query

    # load our yaml file

    with open('location_test.yaml', 'r') as file_stream:
        location_info_yaml = yaml.safe_load(file_stream)
    
    with open('premade_query.yaml', 'r') as file_stream:
        query_info_yaml = yaml.safe_load(file_stream)

    # populate location dictionary like so
    # location name : Location_data object

    for key,value in location_info_yaml["locations"].iteritems():
        trigger = trigger_info(**location_info_yaml["locations"][key])
        trigger_list[key] = trigger

    for key,value in query_info_yaml['queries'].iteritems():
        #query_list[key] = QueryData(**query_info_yaml['queries'][key])
        query_list[key] = value

    # Begin background audio stream
    pbg = multiprocessing.Process(target = bgLoop)
    pbg.start()


    #IGNORING POSE FAKER FOR NOW
    #subscribe to pose_mock
    #publish pose_mock
#    p11 = Process(target = pose_mock(1))
#    p12 = Process(target = pose_mock(2))
#    p11.start()
#    p12.start()

    # Begin monitoring robot location
    p21 = multiprocessing.Process(target = listen_for_pose, args = ('/test/robot1/odometry/filtered',1))
    p22 = multiprocessing.Process(target = listen_for_pose, args = ('/test/robot2/odometry/filtered',2))
    p23 = multiprocessing.Process(target = listen_for_pose, args = ('/test/robot3/odometry/filtered', 3))
    p24 = multiprocessing.Process(target = listen_for_pose, args = ('/test/robot4/odometry/filtered', 4))
    p21.start()
    p22.start()
    p23.start()
    p24.start()
    rospy.spin()


def stream_audio_query(file_path):
    try:
        afile = open(file_path)
    except:
        print("false positive")
        return
    
    bg_playing.value = False
    nbuf = afile.read(4096)
    while True:
        buf = nbuf
        nbuf = afile.read(4096)
        if len(buf) == 0:
            break
        s.send(buf)
        s.sync()
    
    afile.close()
    bg_playing.value = True

def trigger_callback(msg, args):
    print "received"
    #pub_trigger = rospy.Publisher('/audio_trigger',Trigger,queue_size=100 )

    pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    currentzone = ''
    #check if odom x and y are in a trigger zone
    for trigger,tval in trigger_list.iteritems():
        if ( (tval.x < pos[0]) &
                (tval.y < pos[1]) &
                ((tval.x + tval.w) > pos[0]) &
                ((tval.y + tval.h) > pos[1]) ):
            currentzone = trigger
            break
    # if not, leave
    else:
        return

    #check if location has been activated before
    if currentzone in activatedLocations:
        #if so, continue
        return

    #mark location as activated
    activatedLocations[currentzone] = True

    #get query associated with location
    querynum = trigger_list[currentzone].q
    query = query_list['query' + str(querynum)]

    #publish query to topic for logging in audio logger
    audiomsg = AudioDetection()
    audiomsg.robotId = args[0]
    audiomsg.confidence = query['confidence']
    audiomsg.filename = query['file_name']
    audiomsg.transcript = ''
    #audiomsg.robotX = pos[0]
    #audiomsg.robotY = pos[1]

    pub.publish(audiomsg)

    #Cast audio file to icecast
    #TODO: Should this be in a separate process?
    print 'Streaming from source ' + str(query['file_name'])
    stream_audio_query('audio/' + str(query['file_name']))

#publishing node for testing
# def pose_mock(robot_num):

#     pub_test = rospy.Publisher('/pose_mock_'+str(robot_num),Odometry,queue_size = 1000)
#     #rospy.init_node('pose_mock')
#     r = rospy.Rate(10)

#     msg = Odometry()
#     msg.pose.pose.position.x = 10+robot_num
#     msg.pose.pose.position.y = 20+robot_num

#     while not rospy.is_shutdown():
#         pub_test.publish(msg)
#         r.sleep()

def listen_for_pose(topic, robotId):
    rospy.init_node('listen_for_pose', anonymous=True)
    print ("subscribing for " + str(robotId) + " on " + topic)
    rospy.Subscriber(topic, Odometry, trigger_callback, (robotId,))
    print ("subscribed, spinning " + str(robotId))
    rospy.spin()

if __name__ == "__main__":
    main()
