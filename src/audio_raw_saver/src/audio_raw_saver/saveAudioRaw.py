#! /usr/bin/env python2.7

import rospy
import datetime
import os
from audio_common_msgs.msg import AudioData
from detection_msgs.msg import AudioFileReady

today = datetime.date.today()
directory = str(today.year) + '/' + str(today.month) + '/' + str(today.day) + '/audio/'

if not os.path.exists(directory):
  os.makedirs(directory)

FILENAME = directory + 'audio'
fileNum = 0
fileObj = open(FILENAME + str(fileNum) + '.mp3', 'w')
currentSize = 0

FILE_DURATION = 10
AUDIO_BITRATE = 48000

pub = 0

def audioSaveCallback(data):
  global fileObj
  global currentSize
  global fileNum

  for sample in data.data:
    fileObj.write(sample)

  currentSize += 2 * len(data.data) #writing data for 2 channels
  if(currentSize >= AUDIO_BITRATE * FILE_DURATION):
    os.system("ffmpeg -i " + FILENAME + str(fileNum) + ".mp3 -ac 1 " + FILENAME + str(fileNum) + ".wav")
    #construct and send message
    msg = AudioFileReady()
    msg.file_name = 'audio' + str(fileNum) + '.wav'
    pub.publish(msg)

    #Close file and increment number
    fileObj.close()
    fileNum += 1
    fileObj = open(FILENAME + str(fileNum) + '.mp3', 'w')
    currentSize = 0


def listenForAudio():
  global pub

  rospy.init_node('listenForAudio', anonymous=True)
  rospy.Subscriber('/audio/audio', AudioData, audioSaveCallback)

  pub = rospy.Publisher('sarwai_detection/detection_audio_ready', AudioFileReady, queue_size=1000)

  rospy.spin()
