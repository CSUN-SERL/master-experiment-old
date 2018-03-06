#!/usr/bin/env python

from socketIO_client import BaseNamespace
from socketIO_client import LoggingNamespace
from socketIO_client import SocketIO

import rospy
import roslib
import actionlib
import time
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient, GoalStatus
from geometry_msgs.msg import *
#initial position of the robot
robot_number=2
robotx = 0.0962140767671
roboty = -0.557272647468
toggle = False


class Namespace(BaseNamespace):

    def on_cmtogglemanualcontrol(self, *args):
        global toggle
        rospy.loginfo('recieved message')
        print(args[0])
        if (int(args[0]) == robot_number):
            toggle = not toggle

    def on_test(self, *args):
        print('test: ', args[0])
        rospy.loginfo('recieved message')

def distanceToRobot(x1, y1, x2, y2):
    return math.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))


def minDistIndex(list):
    minDistListIndex = 0
    minDist = 999999999999
    for i in list:
        dist = distanceToRobot(i[0][0],i[0][1],robotx,roboty)
        if dist < minDist:
                minDist = dist
                minDistListIndex = list.index(i)
    return list.pop(minDistListIndex)


def talker(x , y , z , w):
    t = int(time.time())
    sac = actionlib.SimpleActionClient('robot2/move_base', MoveBaseAction)
    rospy.loginfo('Sending point')
    sac.wait_for_server()
    goal = MoveBaseGoal()    #use self?
    #set goal
    goal.target_pose.header.frame_id = 'robot2_tf/odom'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w

    #start listner
    #sac.wait_for_server()

    if (toggle):
         print("toggler is toggled")
         sleep(1)
         talker(x ,y, z, w)

    #send goal
    sac.send_goal(goal)
    #finish
    sac.wait_for_result(rospy.Duration(20))

    
    if sac.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo('Done')
    else:
        rospy.loginfo('Fail')
        time.sleep(10)
        sac.cancel_goal()
        sac.send_goal(goal)

def callback(data):
    global robotx
    global roboty
    #location of the robot
    robotx = data.polygon.points[3].x
    roboty = data.polygon.points[3].y
    #rospy.loginfo(robotx)

if __name__ == '__main__':
    try:
        rospy.init_node('talker2', anonymous=True)
        rospy.Subscriber('/robot2/move_base/global_costmap/footprint', PolygonStamped, callback)
        list = [[[-1.04104948044,-3.77272891998,-0.0548761226904,0.998493170311],[-4.72948598862,-3.78209996223,0.999661270513,-0.0260258378574],[-0.983633875847,-3.93218541145,0.75320254302,-0.657788666053]],[[0.625909149647,-8.7917432785,0.999942377211,0.0107350946744],[4.27511930466,-8.65122890472,-1.07930150373e-05,0.999999999942],[0.624517023563,-8.67425060272,-0.495083885352,0.868845179802]],[[-0.957045435905,-11.6428775787,-0.0141909811884,0.999899302957],[-5.13722515106,-11.3043651581,0.999364869076,-0.0356350734137],[-0.550347566605,-11.3650951385,0.74626246275,-0.665651813406]],[[-0.860676586628,-18.2019672394,-0.0814746175342,0.996675416923],[-5.55025911331,-18.0227355957,0.998137372633,-0.0610064369766],[-0.84488093853,-17.9807605743,0.75358806463,-0.657346962301]],[[-0.814701318741,-25.1865501404,0.0356351235341,0.999364867289],[-5.40632724762,-24.725276947,0.997991395161,-0.063349626547],[-0.581647276878,-25.0364398956,0.728432446148,-0.685117633257]],[[0.224287986755,-30.0965175629,0.999743492784,0.0226483694013],[4.92413425446,-29.9763298035,-0.0397200497426,0.999210847443],[0.217172503471,-29.8847904205,0.609984415654,-0.792413410197]],[[-0.959907591343,-32.3575897217,-0.0133071936976,0.999911455378],[-5.18180084229,-32.0116539001,0.999364882495,-0.0356346970633],[-0.429017961025,-32.0842285156,0.704925436874,-0.709281416962]],[[-0.0117795467377,-36.9831848145,0.999942367699,0.0107359806753],[4.90137577057,-36.9893951416,-0.0155784499532,0.999878648585],[0.036406993866,-36.7754058838,0.517176968863,-0.855878486047]],[[-0.988997936249,-38.8366012573,-0.00820061939478,0.999966374355],[-6.05384540558,-38.4637985229,0.997020570653,-0.0771361244476],[-0.967669665813,-38.6935806274,0.721424313586,-0.692493292219]],[[-0.116713762283,-43.2788581848,0.999903159039,-0.0139166283532],[5.01764965057,-43.3008651733,0.0244064238862,0.99970211887],[-0.413137376308,-43.1262321472,0.65876888109,-0.752345373686]],[[-1.1864181757,-46.0535011292,-0.00968721709364,0.999953077812],[-6.35236406326,-45.7007408142,0.998780023475,-0.0493808131497],[-1.05351877213,-45.9985046387,0.739984377701,-0.672624056036]],[[-0.372879087925,-50.4643936157,0.9997995392,-0.0200220232541],[4.52497339249,-50.7335243225,0.0129305721163,0.999916396658],[-0.447350919247,-50.3784675598,0.681459896909,-0.731855456293]],[[-1.42113947868,-52.7271652222,-0.000877599395792,0.99999961491],[-5.66937971115,-52.6172714233,0.999364853463,-0.0356355112658],[-1.34666800499,-52.8130912781,0.692807824436,-0.721122263143]],[[-0.621347844601,-57.104850769,0.999673800813,-0.0255400072098],[4.18026018143,-57.1414070129,-0.0229096817329,0.999737538799],[-0.797790229321,-56.8665466309,0.630890164989,-0.775872154237]],[[-1.88873708248,-60.3265037537,0.0356354130188,0.999364856966],[-6.01226902008,-59.8720474243,0.999553148297,-0.0298915327536],[-1.31851434708,-60.2316627502,0.668288782358,-0.743901944731]],[[-0.988248944283,-64.1677322388,0.997991375812,-0.0633499313727],[3.93228459358,-64.4335021973,-0.0259867957749,0.999662286197],[-1.39487361908,-64.896736145,0.655736274907,-0.754990024948]],[[-1.85533761978,-67.0646286011,-0.0462772877202,0.998928632406],[-7.45534515381,-66.7258911133,0.999999264012,-0.00121325002208],[-1.75020384789,-66.962097168,0.681459724122,-0.731855617181]],[[-2.45733141899,-73.9540328979,0.0148205316228,0.99989016989],[-7.47431230545,-73.6691436768,0.99936483342,-0.0356360733352],[-2.10202097893,-73.5563354492,0.999364849439,-0.0356356241038]],[[0.82354080677,-74.5409545898,0.730122551026,-0.683316222903],[0.642064511776,-77.456993103,0.737702328521,0.675126117474],[0.894823908806,-74.4791717529,0.934905567114,0.354896577301]],[[3.10464000702,-73.5001373291,0.731855606987,0.68145973507],[3.21007990837,-70.7124176025,0.728706033374,-0.68482663275],[2.86322450638,-73.6968917847,0.999943146281,-0.010663217437]],[[7.47301483154,-74.6947860718,0.717853963243,-0.696193714031],[7.25447463989,-77.7552947998,0.713359657327,0.700798115935],[7.26295185089,-74.2651748657,0.997830669111,0.0658327865213]],[[11.9135742188,-73.5674285889,0.727415682858,0.686197073976],[12.0902996063,-70.7179260254,0.721424252131,-0.692493356241],[11.7529468536,-73.5693588257,0.999553149967,-0.0298914769096]],[[15.2525863647,-74.3809814453,0.69624455905,-0.717804648908],[15.1262569427,-78.2104034424,0.705675776382,0.708534895843],[14.9275178909,-74.0635223389,0.00191073689231,0.999998174541]]]

        for i in range(0,len(list)):
		    #nearRoom = nearest outside point's room in the map from the robot's position
			nearRoom = minDistIndex(list)
			for j in nearRoom:
				talker(j[0],j[1],j[2],j[3])
    except rospy.ROSInterruptException:
        pass
