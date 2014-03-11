#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, tty, termios, signal
import numpy as np
import math
from std_msgs.msg import Int32
#CONSTANTE
VALEUR_X = 0.2
TIMEOUT_USER = 3

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
status_robot = Status.NORMAL
pos_obstacle = PosObstacle.RIEN
num_user = 0
timer = 0

#FUNCTION
"""def callbackInfra(msg):
    pos_obstacle = msg"""

def stopFollowingTimeOut(event):
     global timer, status_robot, num_user
     status_robot = Status.NORMAL
     num_user = 0
     timer.shutdown()

def callbackKinectNewUser(msg):
    global status_robot
    global num_user, timer
    if status_robot == Status.NORMAL:
        status_robot = Status.SQUELETTE
        num_user = msg.data
        timer = rospy.Timer(rospy.Duration(TIMEOUT_USER), stopFollowingTimeOut)

def callbackKinectDeleteUser(msg):
    global status_robot
    global num_user
    if status_robot == Status.SQUELETTE and msg.data == num_user:
        status_robot = Status.NORMAL
        num_user = 0
    	
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
    rospy.Subscriber("/isUserVisible", Int32, callbackKinectNewUser) #callback kinect squelette detection
    rospy.Subscriber("/userDisapeared", Int32, callbackKinectDeleteUser) #callback kinect squelette detection
    #rospy.Subscriber("camera/depth/image", Type??, callbackBalance) #callback balance
    #rospy.Subscriber("camera/depth/image", Type??, callbackUltraSon) #callback ultrason
    
    #departBase()    
    global status_robot, pos_obstacle
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
                twist = Twist()s
                twist.angular.z = 4.0
                pub.publish(twist)"""
        #elif status_robot == Status.RETOUR:
            #code retour base

if __name__ == '__main__':
    num_user = 0
    pos_obstacle = PosObstacle.RIEN
    try:
        navigationturtle()
    except rospy.ROSInterruptException:
        pass

