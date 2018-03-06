#! /usr/bin/env python2.7


import rospy
import datetime
import os
#from audio_common_msgs.msg import AudioData
from detection_msgs.msg import AudioFileReady
from detection_msgs.msg import AudioDetection

#google cloud import
# [START import_libraries]
import argparse
import io
import google.cloud
# [END import_libraries]
os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = "/home/serl/Documents/Detection/Detection-SARWAI/HARK to text-6c6fe313325d.json"

today = datetime.date.today()
directory = str(today.year) + '/' + str(today.month) + '/' + str(today.day) + '/audio/'

pub = 0

def transcribe_file(data):
    global pub
    print 'running'
    """Transcribe the given audio file."""
    from google.cloud import speech
    from google.cloud.speech import enums
    from google.cloud.speech import types
    client = speech.SpeechClient()

    #get audio file name
    speech_file = directory+data.file_name
    # [START migration_sync_request]
    # [START migration_audio_config_file]
    with io.open(speech_file, 'rb') as audio_file:
        content = audio_file.read()
    print speech_file
    audio = types.RecognitionAudio(content=content)
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=44100,
        language_code='en-US')
    # [END migration_audio_config_file]

    # [START migration_sync_response]
    response = client.recognize(config, audio)
    # [END migration_sync_request]
    # Print the first alternative of all the consecutive results.
    for result in response.results:
        msg = AudioDetection()
        msg.transcript = result.alternatives[0].transcript
        msg.confidence = result.alternatives[0].confidence
        msg.filename = speech_file
        print('Transcript: {}'.format(result.alternatives[0].transcript))
        print('Confidence: {}'.format(result.alternatives[0].confidence))
        pub.publish(msg)


def listen_for_file():
  global pub
  print 'running'
  rospy.init_node('listenForAudioFile', anonymous=True)
  rospy.Subscriber('sarwai_detection/detection_audio_ready', AudioFileReady, transcribe_file)

  pub = rospy.Publisher('sarwai_detection/detection_audio_transcript', AudioDetection, queue_size=1000)

  rospy.spin()
