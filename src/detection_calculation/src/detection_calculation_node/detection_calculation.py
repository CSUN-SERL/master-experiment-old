#!/usr/bin/env python

import rospy
import yaml
#import sys
import time
import tf
import numpy
#from rospy import ROSException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
#from geometry_msgs.msg import Pose
from detection_msgs.msg import CompiledFakeMessage
from detection_msgs.msg import Human
import math
from sympy import Segment as Segment
from sympy import Triangle as Triangle

import multiprocessing
import os.path

import pickle



global MyHumans
global init_robot_pose

print "ASSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS SS SS"

#Config file dictinary
MyHumans = yaml.load(open('/home/serl/sarwai-experiment-fd/human.yaml'))
init_robot_pose = yaml.load(open('/home/serl/sarwai-experiment-fd/robot.yaml'))
walls_yaml = yaml.load(open('/home/serl/sarwai-experiment-fd/walls.yaml'))

global max_distance

class Wall():
	def __init__(self, id = -1, segment1 = Segment((0,0),(0,0)), segment2 = Segment((0,0),(0,0)), segment3 = Segment((0,0),(0,0)), segment4 = Segment((0,0),(0,0))):
		self.id = id
		self.segment1 = segment1
		self.segment2 = segment2
		self.segment3 = segment3
		self.segment4 = segment4

#Generate list of Walls
global walls_list
walls_list = []

global counter 
counter = 0

import os
dir_path = os.path.dirname(os.path.realpath(__file__))
pickle_loc = '/home/serl/sarwai-experiment-fd/src/detection_calculation/src/detection_calculation_node/wall_seg_pickle.obj'
if not os.path.isfile(pickle_loc):
	for wall_id,seg_list in walls_yaml.iteritems():
		seg1 = Segment(tuple(seg_list['p1']), tuple(seg_list['p2']))
		seg2 = Segment(tuple(seg_list['p2']), tuple(seg_list['p4']))
		seg3 = Segment(tuple(seg_list['p3']), tuple(seg_list['p1']))
		seg4 = Segment(tuple(seg_list['p4']), tuple(seg_list['p3']))
		walls_list.append(Wall(wall_id, seg1, seg2, seg3, seg4))

	with open('wall_seg_pickle.obj', 'w') as pickle_dest:
		pickle.dump(walls_list,pickle_dest)

	print 'done parsing'
else:
	print 'File found successfully'

with open(pickle_loc) as pickle_src:
	walls_list = pickle.load(pickle_src) 

# print walls_list



class QTreeNode():
	def __init__(self, left_x, right_x, bottom_y, top_y):
		self.edge_walls = []

		self.left_x = left_x
		self.right_x = right_x
		self.top_y = top_y
		self.bottom_y = bottom_y

		self.tl = None
		self.tr = None
		self.bl = None
		self.br = None

	def InsertWall(self, wall):
		#get center point
		diagonal = Segment((self.left_x, self.top_y), (self.right_x, self.bottom_y))
		center_x = diagonal.midpoint.x
		center_y = diagonal.midpoint.y

		#Init subnodes
		if self.tl == None:
			self.tl = QTreeNode(self.left_x, center_x, center_y, self.top_y)
			self.tr = QTreeNode(center_x, self.right_x, center_y, self.top_y)
			self.bl = QTreeNode(self.left_x, center_x, self.bottom_y, center_y)
			self.br = QTreeNode(center_x, self.right_x, self.bottom_y, center_y)

		nodes = []
		wall_segments = [wall.segment1, wall.segment2, wall.segment3, wall.segment4]

		for seg in wall_segments:
			# if segment exists in left nodes
			if seg.p1.x <= center_x or seg.p2.x <= center_x:
				# Check if segment exists in top node
				if seg.p1.y >= center_y or seg.p2.y >= center_y:
					nodes.append(self.tl)
				# Check if segment exists in bottom node
				if seg.p1.y <= center_y or seg.p2.y <= center_y:
					nodes.append(self.bl)
			# If segment exists in right nodes
			if seg.p1.x >= center_x or seg.p2.x >= center_x:
				# Check if segment exists in top node
				if seg.p1.y >= center_y or seg.p2.y >= center_y:
					nodes.append(self.tr)
				# Check if segment exists in bottom node
				if seg.p1.y <= center_y or seg.p2.y <= center_y:
					nodes.append(self.br)

		if len(nodes) > 1:
			#On edge, put in edge_walls
			for node in nodes:
				node.edge_walls.append(wall)
		else:
			#Within a node, insert into the subnode
			nodes[0].InsertWall(wall)

	def GetChildWalls(self):
		ret = self.edge_walls[:]
		print len(ret)
		if self.tl != None:
			ret += self.tl.GetChildWalls()[:] + self.tr.GetChildWalls()[:] + self.bl.GetChildWalls()[:] + self.br.GetChildWalls()[:]
		return ret

class QTree():
	def __init__(self, x_min, x_max, y_min, y_max):
		self.root = QTreeNode(x_min, x_max, y_min, y_max)

	def populate(self, walls):
		for wall in walls:
			self.root.InsertWall(wall)

	def GetReleventWallsShitty(self, robot_pos, robot_theta):
		current_node = self.root
		half_fov_angle = robot_fov / 2.0
		leg_length = max_distance / math.cos(half_fov_angle)
		fov = Triangle(robot_pos, (robot_pos[0] + leg_length*math.cos(robot_theta - half_fov_angle), robot_pos[1] + leg_length*math.sin(robot_theta - half_fov_angle)), (robot_pos[0] + leg_length*math.cos(robot_theta + half_fov_angle), robot_pos[1] + leg_length*math.sin(robot_theta + half_fov_angle)))
		# Check if fov is hitting a center edge
		while True:
			#get center point
			diagonal = Segment((current_node.left_x, current_node.top_y), (current_node.right_x, current_node.bottom_y))
			center_x = diagonal.midpoint.x
			center_y = diagonal.midpoint.y

			node_vertical = Segment((center_x, current_node.top_y), (center_x, current_node.bottom_y))
			node_horizontal = Segment((current_node.left_x, center_y), (current_node.right_x, center_y))
			if len(node_vertical.intersection(fov) + node_horizontal.intersection(fov)) >= 0:
				return current_node.GetChildWalls()
			elif current_node.tl == None:
				return []
			else:
				if robot_pos[0] < center_x:
					if robot_pos[1] < center_y:
						current_node = current_node.br
					else:
						current_node = current_node.tr
				else:
					if robot_pos[1] < center_y:
						current_node = current_node.bl
					else:
						current_node = current_node.tl


global world_qtree
world_qtree = QTree(x_min=-166.0, x_max=166.0, y_min=-148.0, y_max=168)

def process():
#RosLaunch Parameters
	print('^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^STARTING TO SLEEP BITCH')
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

	world_qtree.populate(walls_list)
	print 'PENIS'

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
			if (human_angle <= fov_offset ) and (human_angle >= (fov_offset * -1)) and (MyHumans[human_num]['dclass'] != 2) and in_view(RoboPosX, RoboPosY, RoboPosTh, MyHumans[human_num]['x'], MyHumans[human_num]['y']):
				man = Human()
				man.id = i
				man.dclass = MyHumans[human_num]['dclass']
				man.angleToRobot = human_angle
				man.distanceToRobot = dist
				ret.append(man)
	return ret

def get_incident_walls(robot_pos, robot_theta):
	return world_qtree.GetReleventWallsShitty(robot_pos, robot_theta)


def in_view(RoboPosX, RoboPosY,RoboPosTh,HumanX,HumanY):
	line_of_sight = Segment((RoboPosX,RoboPosY),(HumanX,HumanY))
	wall_segments = [] # get_incident_walls( (RoboPosX,RoboPosY), RoboPosTh)
	from rtree import index

	segments = []
	for wall in wall_list:
		segments.append(wall.segment1)
		segments.append(wall.segment2)
		segments.append(wall.segment3)
		segments.append(wall.segment4)

		for segment in segments:
			if line_of_sight.intersection(segment) == []:
				# print line_of_sight.intersection(segment)
				return True
		segments = []


	return False

def main():
	process()
