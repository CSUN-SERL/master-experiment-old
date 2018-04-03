#!/usr/bin/env python

import rospy
import yaml
import time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from detection_msgs.msg import CompiledFakeMessage
from detection_msgs.msg import Human

import human_finder
from util import *

global FOV_MARGIN
FOV_MARGIN = 0.087 / 2.0


def Odometry_update(data, force_detection=False):
    # Getting x and z change for robot
    x = data.pose.pose.position.x * -1
    y = data.pose.pose.position.y * -1

    # Getting the Quaternion info
    xQ = data.pose.pose.orientation.x
    yQ = data.pose.pose.orientation.y
    zQ = data.pose.pose.orientation.z
    wQ = data.pose.pose.orientation.w

    # Converting quaternion to euler angle
    xE, yE, zE = quaternion_to_euler_angle(xQ, yQ, zQ, wQ)

    # Robot position constantly updated
    new_x_pos = robot_pos_x + x
    new_y_pos = robot_pos_y + y
    new_th_pos = robot_pos_th + zE

    # Searching for humans
    # O(n) search
    # humans within fov

    humans_in_view_dict = human_detector.find_people_in_view(new_x_pos, new_y_pos, new_th_pos, force_detection)

    humans_list = []

    for human_id, human_data in humans_in_view_dict.iteritems():
        human = Human()
        human.id = int(human_id)
        human.dclass = human_data['dclass']
        human.angleToRobot = human_data['human_angle']
        human.distanceToRobot = human_data['distance_to_robot']
        human.confidence = human_data['confidence']
        human.forced = force_detection

        humans_list.append(human)

    # Msgs being set and released
    new_message = CompiledFakeMessage()
    new_message.humans = humans_list
    new_message.header.stamp = rospy.Time.now()
    new_message.robot = robot_number
    # float
    new_message.fov = robot_fov

    new_message.img = rospy.wait_for_message('/robot' + str(robot_number) + '/camera/rgb/image_raw', Image,
                                             timeout=None)
    pub.publish(new_message)

def Force_update(data):
    print "force detection"
    if int(data.data) == robot_number:
        odom_data = rospy.wait_for_message('/robot' + str(robot_number) + '/odom', Odometry, timeout=None)
        Odometry_update(odom_data, force_detection=True)

def main():
    rospy.init_node("detection_calculation", anonymous=True)

    global mission_number
    mission_number = rospy.get_param('~mission_number')
    global robot_number
    robot_number = rospy.get_param('~robot_number')

    global init_robot_pose
    init_robot_pose = yaml.load(open('robot.yaml'))

    global robot_pos_x
    robot_pos_x = init_robot_pose[mission_number][str(robot_number)]['x']

    global robot_pos_y
    robot_pos_y = init_robot_pose[mission_number][str(robot_number)]['y']

    global robot_pos_th
    robot_pos_th = init_robot_pose[mission_number][str(robot_number)]['theta']

    global depth_of_field
    depth_of_field = init_robot_pose[mission_number][str(robot_number)]['dof']

    global robot_fov
    robot_fov = init_robot_pose[mission_number][str(robot_number)]['fov']

    global humans_dict
    humans_dict = yaml.load(open('human.yaml'))

    global walls_dict
    walls_dict = yaml.load(open('walls.yaml'))

    global human_detector
    human_detector = human_finder.HumanFinder(walls_dict, humans_dict, depth_of_field, robot_fov, FOV_MARGIN, robot_number)

    #print('WAITING FOR GAZEBO')
    # time.sleep(35)
    print("START CALCULATION")

    global pub
    pub = rospy.Publisher('sarwai_detection/custom_msgs_info', CompiledFakeMessage, queue_size=1000)

    rospy.Subscriber('robot' + str(robot_number) + '/odom', Odometry, Odometry_update, queue_size=1)

    rospy.Subscriber('coffee', String, Force_update, queue_size=1)

    rospy.spin()
