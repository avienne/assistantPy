#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, tty, termios
import numpy as np
import math
from std_msgs.msg import Int32
#CONSTANTE
VALEUR_X = 0.2

#ENUM
class Status:
    NORMAL = 0
    RETOUR = 1
    SQUELETTE = 2

class PosObstacle:
    RIEN = 0
    AV_GAUCHE = 1
    AV_DROITE = 2
    GAUCHE = 3
    DROITE = 4

#GLOBAL
#global status_robot
status_robot = Status.NORMAL
#global num_user
#global pos_obstacle
pos_obstacle = PosObstacle.RIEN
num_user = 0

#FUNCTION
def convertToRad(degree):
    return degree * math.pi * 1.0 / 180

def convertToDegree(rad):
    return rad * 180.0 / math.pi

"""def callbackInfra(msg):
    pos_obstacle = msg"""

def callbackKinectUser(msg):
    global status_robot
    global num_user
    if status_robot == Status.NORMAL and msg.data != 0:
        status_robot = Status.SQUELETTE
        num_user = msg.data
    	print "%d" % status_robot

"""def callbackBalance(msg):
DEFINIR CE QUE LA BALANCE RENVOIT (poid ? mais si plateau ou autre vide != 0 non ?)
"""

"""def callbackUltraSon(msg):
DEFINIR CE QUE LES ULTRASONS RENVOIENT (que a partir d'une certaine distance pour detecter une personne seulement ??)
"""

def departBase():
    """On sort du dock"""
    twist = Twist()
    twist.linear.x = -0.2
    pub.publish(twist)

    """On fait demi-tour"""
    twist = Twist()
    twist.angular.z = 4.0
    pub.publish(twist)


def navigationturtle():
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
    rospy.init_node('navigationturtle')

    #rospy.Subscriber("camera/depth/image", Type??, callbackinfra) #callback capteur infrarouge1
    rospy.Subscriber("/isUserVisible", Int32, callbackKinectUser) #callback kinect squelette detection
    #rospy.Subscriber("camera/depth/image", Type??, callbackBalance) #callback balance
    #rospy.Subscriber("camera/depth/image", Type??, callbackUltraSon) #callback ultrason
    
    print "navg"
    
    #departBase()    
    global status_robot
    while not rospy.is_shutdown():
        if status_robot == Status.SQUELETTE:
            if pos_obstacle == PosObstacle.RIEN:
                twist = Twist()
                twist.linear.x = VALEUR_X
                pub.publish(twist)
        elif status_robot == Status.NORMAL:
	    """if pos_obstacle != PosObstacle.AV_GAUCHE and pos_obstacle != PosObstacle.AV_DROITE:
                twist = Twist()
                twist.linear.x = VALEUR_X
                pub.publish(twist)
            elif pos_obstacle != PosObstacle.AV_GAUCHE and pos_obstacle != PosObstacle.GAUCHE:
                twist = Twist()
                twist.angular.z = -2.0
                pub.publish(twist)
            elif pos_obstacle != PosObstacle.AV_DROITE and pos_obstacle != PosObstacle.DROITE:
                twist = Twist()
                twist.angular.z = 2.0
                pub.publish(twist) 	    
            else:
                twist = Twist()
                twist.angular.z = 4.0
                pub.publish(twist)"""
            #elif status_robot == Status.RETOUR:
            #code retour base	
        rospy.sleep(1.0)

if __name__ == '__main__':
    num_user = 0
    pos_obstacle = PosObstacle.RIEN
    try:
        navigationturtle()
    except rospy.ROSInterruptException:
        pass

