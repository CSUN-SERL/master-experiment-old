#!/usr/bin/env python

import rospy
import yaml
import sys
import time
import tf
import numpy
from rospy import ROSException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from detection_msgs.msg import CompiledFakeMessage
from detection_msgs.msg import Human
import math


global MyHumans
global init_robot_pose

image_arr = []


#Config file dictinary
MyHumans = yaml.load(open('/home/serl/sarwai-experiment-fd/human.yaml'))
init_robot_pose = yaml.load(open('/home/serl/sarwai-experiment-fd/robot.yaml'))

global max_distance

def process():
#RosLaunch Parameters
	time.sleep(35)
	print("##########################START CALCULATION")
	rospy.init_node("detection_calculation", anonymous=True)

	global mission_number_
	global robot_number_
	mission_number_ = rospy.get_param('~mission_number')
	robot_number_ = rospy.get_param('~robot_number')

	global robot_pos_x
	global robot_pos_y
	global robot_pos_th
	robot_pos_x = init_robot_pose[mission_number_][str(robot_number_)]['x']
	robot_pos_y = init_robot_pose[mission_number_][str(robot_number_)]['y']
	robot_pos_th = init_robot_pose[mission_number_][str(robot_number_)]['theta']

	global max_distance
	global robot_fov
	max_distance = init_robot_pose[mission_number_][str(robot_number_)]['dof']
	robot_fov = init_robot_pose[mission_number_][str(robot_number_)]['fov']

	global FOV_MARGIN
	FOV_MARGIN = 0.087 / 2.0

	global pub
	pub = rospy.Publisher('sarwai_detection/custom_msgs_info', CompiledFakeMessage, queue_size=1000)

	rospy.Subscriber('robot' + str(robot_number_) + '/odom', Odometry, Odometry_update, queue_size=1)
	
	rospy.Subscriber('coffee', String, Force_update, queue_size = 1)

	rospy.spin()

def Force_update(data):
	print("Got force")

def Odometry_update(data):
	#Getting x and z change for robot
	x = data.pose.pose.position.x * -1
	y = data.pose.pose.position.y * -1

	#Getting the Quaternion info
	xQ = data.pose.pose.orientation.x
	yQ = data.pose.pose.orientation.y
	zQ = data.pose.pose.orientation.z
	wQ = data.pose.pose.orientation.w

	# Converting quaternion to euler angle
	xE,yE,zE = quaternion_to_euler_angle(xQ,yQ,zQ,wQ)


	#Robot position constantly updated
	new_x_pos = robot_pos_x + x
	new_y_pos = robot_pos_y + y
	new_th_pos = robot_pos_th + zE

	#Searching for humans
	human_list = find(new_x_pos, new_y_pos, new_th_pos)

	#Msgs being set and released
	new_message = CompiledFakeMessage()
	new_message.humans = human_list
	new_message.header.stamp = rospy.Time.now()
	new_message.robot = robot_number_
	new_message.fov = robot_fov

	new_message.img = rospy.wait_for_message('/robot' + str(robot_number_) + '/camera/rgb/image_raw', Image, timeout=None)
	pub.publish(new_message)

def quaternion_to_euler_angle(x, y, z, w):
	quaternion = (x, y, z, w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	return euler[0], euler[1], euler[2]


def shift_points(RX,RY,HX,HY):
	HX = HX - RX
	HY = HY - RY

	return HX,HY


def cartesian_to_polar_distance(x,y):
	return math.sqrt(x**2 + y**2)


#returning rad
def cartesian_to_polar_angle(x,y):
	return math.atan2(y,x)

def normalize_to_angle(vecX, vecY, theta):
  vecArray = numpy.matrix([[float(vecX)], [float(vecY)]])
  theta *= -1.0
  thetaCos = math.cos(theta)
  thetaSin = math.sin(theta)
  rotationMatrix = numpy.matrix([[thetaCos, -1 * thetaSin], [thetaSin, thetaCos]])

  rotation = numpy.matmul(rotationMatrix, vecArray)
  return rotation[0,0],rotation[1,0]

def find(RoboPosX, RoboPosY, RoboPosTh):
	ret = []
	for i in range(291):
		human_num = str(i)

		# Get the coordinates of the human relative to the robot
		hx,hy = shift_points(RoboPosX, RoboPosY, MyHumans[human_num]['x'], MyHumans[human_num]['y'])

		# Get the distance between the humann and the robot
		dist = cartesian_to_polar_distance(hx, hy)

		if dist <= max_distance:  #dof
			# Rotoate the human and the robot so the robot is facing X=0, and get the human's new coordinates
			relHX,relHY = normalize_to_angle(hx, hy, RoboPosTh)

			# Get the new angle between the robot and the human
			human_angle = cartesian_to_polar_angle(relHX, relHY)

			fov_offset = (robot_fov / 2.0) - FOV_MARGIN

			# If the human has an angle less than half the robot's FOV (fov_offset)...
			if (human_angle <= fov_offset ) and (human_angle >= (fov_offset * -1)) and (MyHumans[human_num]['dclass'] != 2):
				man = Human()
				man.id = i
				man.dclass = MyHumans[human_num]['dclass']
				man.angleToRobot = human_angle
				man.distanceToRobot = dist
				ret.append(man)
	return ret


def main():
	process()


if __name__ == "__main__":
	main()
