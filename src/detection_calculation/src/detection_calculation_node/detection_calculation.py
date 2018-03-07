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

human_msg_ = Human()
compiled_msgs_ = CompiledFakeMessage()


#Config file dictinary
MyHumans = yaml.load(open('/home/serl/sarwai-experiment/human.yaml'))
init_robot_pose = yaml.load(open('/home/serl/sarwai-experiment/robot.yaml'))

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

	global pub
	pub = rospy.Publisher('sarwai_detection/custom_msgs_info', CompiledFakeMessage, queue_size=1000)
	
	global imgpub
	imgpub = rospy.Publisher('test_img_topic', Image, queue_size = 100)
	rospy.Subscriber('robot' + str(robot_number_) + '/odom', Odometry, Odometry_update, queue_size=1)
	
	rospy.Subscriber('coffee', String, Force_update, queue_size = 1)

	global counter
	counter = 0

	rospy.spin()

def Force_update(data):
	print("Got force")

def Odometry_update(data):
	# print('******************************ODOM UPDAAAAATTTEEEEE')
	#Getting x and z change for robot
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y

	#Getting the Quaternion info
	xQ = data.pose.pose.orientation.x
	yQ = data.pose.pose.orientation.y
	zQ = data.pose.pose.orientation.z
	wQ = data.pose.pose.orientation.w

	# Converting quaternion to euler angle
	xE,yE,zE = quaternion_to_euler_angle(xQ,yQ,zQ,wQ)

	#Robot position constantly updated
	#global robot_pos_x, robot_pos_y, robot_pos_th
	new_x_pos = robot_pos_x + x
	new_y_pos = robot_pos_y + y
	new_th_pos = robot_pos_th + zE

	#Searching for humans
	find(new_x_pos, new_y_pos, new_th_pos)

	#Msgs being set and released
	compiled_msgs_.header.stamp = rospy.Time.now()

	compiled_msgs_.img =  rospy.wait_for_message('/robot' + str(robot_number_) + '/camera/rgb/image_raw', Image, timeout=None)

	global counter

	if robot_number_ == 3:
		#print(str(new_th_pos))
		imgpub.publish(compiled_msgs_.img)
		#counter = 0
	#counter += 1
		
	compiled_msgs_.robot = robot_number_
	compiled_msgs_.fov = robot_fov
	pub.publish(compiled_msgs_)
	compiled_msgs_.humans = []


#Conversion Function 
# def quaternion_to_euler_angle(x, y, z, w):
# 	ysqr = y * y

# 	t0 = +2.0 * (w * x + y * z)
# 	t1 = +1.0 - 2.0 * (x * x + ysqr)
# 	X = math.atan2(t0, t1)

# 	t2 = +2.0 * (w * y - z * x)
# 	t2 = +1.0 if t2 > +1.0 else t2
# 	t2 = -1.0 if t2 < -1.0 else t2
# 	Y = math.asin(t2)

# 	t3 = 2.0 * (w * z + x * y)
# 	t4 = 1.0 - (2.0 * (ysqr + z * z))
# 	Z = math.atan2(t3, t4)
	
# 	return X, Y, Z

def quaternion_to_euler_angle(x, y, z, w):
	quaternion = (x, y, z, w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	# if robot_number_ == 3:
	# 	print(str(euler[0]) + ' ' + str(euler[1]) + ' ' + str(euler[2]))
	return euler[0], euler[1], euler[2]


def shift_points(RX,RY,HX,HY):
	HX = HX - RX
	HY = HY - RY

	return HX,HY


def cartesian_to_polar_distance(x,y):
	return math.sqrt(x**2 + y**2)


#returning rad
def cartesian_to_polar_angle(x,y):
	return math.atan(y/x)

def normalize_to_angle(vecX, vecY, theta):
	vecArray = numpy.array([vecX, vecY])
	theta *= -1.0
	thetaCos = math.cos(theta)
	thetaSin = math.sin(theta)
	rotationMatrix = numpy.array([[thetaCos, -1 * thetaSin], [thetaSin, thetaCos]])

	rotation = numpy.matmul(vecArray, rotationMatrix)
	return rotation[0],rotation[1]

def find(RoboPosX, RoboPosY, RoboPosTh):
	for i in range(291):
		human_num = str(i)
		hx,hy = shift_points(RoboPosX, RoboPosY, MyHumans[human_num]['x'], MyHumans[human_num]['y'])
		dist = cartesian_to_polar_distance(hx, hy)
		if dist <= max_distance:  #dof
			# hx,hy = shift_points(RoboPosX,RoboPosY, MyHumans[human_num]['x'], MyHumans[human_num]['y'])
			relHX,relHY = normalize_to_angle(hx, hy, RoboPosTh)
			human_angle = cartesian_to_polar_angle(relHX, relHY)
			human_angle *= -1
			fov_offset = robot_fov / 2.0

			# if robot_number_ == 3:
			# 	print('robot ' + str(robot_angle_lower) + ', ' + str(robot_angle_upper) + ' PERSON ' + str(human_angle))
			if (human_angle <= fov_offset ) and (human_angle >= (fov_offset * -1)) and (MyHumans[human_num]['dclass'] != 2):
				# if robot_number_ == 3:
				# 	print('Adding human')
				if robot_number_ == 3:
					print "robot x : ",RoboPosX
					print "robot y : ",RoboPosY 
					# print "human x : ",MyHumans[human_num]['x'] 
					# print "human y : ",MyHumans[human_num]['y']
					print 'human x : ',hx
					print 'human y : ',hy
					print "8====D"
				human_msg_.id = i
				human_msg_.dclass = MyHumans[human_num]['dclass']
				human_msg_.angleToRobot = human_angle
				human_msg_.distanceToRobot = dist
				compiled_msgs_.humans.append(human_msg_)
				break
	# if robot_number_ == 3:
	# 	print('---')



def main():
	process()


if __name__ == "__main__":
	main()
