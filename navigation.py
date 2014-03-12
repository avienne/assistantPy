#/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, tty, termios, signal
import numpy as np
import math
from std_msgs.msg import Int32
from sensor_msgs.msg import Range

#CONSTANTE
VALEUR_X = 0.2
VALEUR_ANGLE = 1
TIMEOUT_USER = 10
ULTRASON_MAX_DIST = 150
ULTRASON_ERR = 20

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

class Direction : 
    NORD = 0
    EST = 1
    OUEST = 2
    NE = 3
    NO = 4
class CurrentUltrasoundMesurement :
    timestamp = 0
    ultrasonAvant = 0
    ultrasonGauche = 0
    ultrasonDroit = 0
 
#GLOBAL
status_robot = Status.SQUELETTE
pos_obstacle = PosObstacle.RIEN
num_user = 0
timer = 0
direction = Direction.NORD
currentUs = CurrentUltrasoundMesurement()

#FUNCTION
"""def callbackInfra(msg):
    pos_obstacle = msg"""

def stopFollowingTimeOut(event):
     global timer, status_robot, num_user
     status_robot = Status.NORMAL
     num_user = 0
     rospy.logdebug("T O")
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
	rospy.logdebug("Del User")
    	
"""def callbackBalance(msg):
DEFINIR CE QUE LA BALANCE RENVOIT (poid ? mais si plateau ou autre vide != 0 non ?)
"""

def callbackUltrasonAvant(msg):
    global status_robot, direction, currentUs
    if status_robot == Status.SQUELETTE :
        if msg.header.stamp.secs >  currentUs.timestamp :
	    currentUs.timestamp = msg.header.stamp.secs
            currentUs.ultrasonAvant = msg.range
            rospy.logdebug("MAJ seq : %d US avant : %d", msg.header.stamp.secs, msg.range)
        elif msg.header.stamp.secs ==  currentUs.timestamp :
            currentUs.ultrasonAvant = msg.range
            rospy.logdebug("new UsAvant : %d", msg.range)
            majDirection()
        else :
            rospy.logdebug("outdated usavant")

def callbackUltrasonGauche(msg):
    global status_robot, direction, currentUs
    if status_robot == Status.SQUELETTE :
        if  msg.header.stamp.secs >  currentUs.timestamp :
	    currentUs.timestamp = msg.header.stamp.secs
            currentUs.ultrasonGauche = msg.range
            rospy.logdebug("MAJ seq : %d US Gauche : %d", msg.header.stamp.secs, msg.range)
        elif msg.header.stamp.secs ==  currentUs.timestamp :
            currentUs.ultrasonGauche = msg.range
            rospy.logdebug("new UsGauche : %d", msg.range)
            majDirection()
        else : 
            rospy.logdebug("outdated usgauche")

def callbackUltrasonDroit(msg):
    global status_robot, direction, currentUs
    if status_robot == Status.SQUELETTE :
        if msg.header.stamp.secs >  currentUs.timestamp :
	    currentUs.timestamp = msg.header.stamp.secs
            currentUs.ultrasonDroit = msg.range
            rospy.logdebug("MAJ seq : %d US Droit : %d", msg.header.stamp.secs, msg.range)
        elif msg.header.seq ==  currentUs.timestamp :
            currentUs.ultrasonDroit = msg.range
            rospy.logdebug("new UsDroit : %d", msg.range)
            majDirection()
        else :
            rospy.logdebug("outdated usDroit")
	    
def majDirection() :
    global direction, currentUs
    if currentUs.ultrasonGauche > currentUs.ultrasonAvant - ULTRASON_ERR and currentUs.ultrasonGauche < currentUs.ultrasonAvant + ULTRASON_ERR :
	direction = Direction.NO
    elif currentUs.ultrasonDroit > currentUs.ultrasonAvant - ULTRASON_ERR and currentUs.ultrasonDroit < currentUs.ultrasonAvant + ULTRASON_ERR :
	direction = Direction.NE
    else :
	direction = Direction.NORD
    rospy.logdebug("%d", direction)
       
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
    rospy.init_node('navigationturtle', log_level=rospy.DEBUG)

    # rospy.Subscriber("/isUserVisible", Int32, callbackKinectNewUser) #callback kinect squelette detection
    rospy.Subscriber("/userDisapeared", Int32, callbackKinectDeleteUser) #callback kinect squelette detection
    rospy.Subscriber("/US1", Range, callbackUltrasonAvant) #callback Ultrason1
    rospy.Subscriber("/US2", Range, callbackUltrasonGauche) #callback Ultrason1
    rospy.Subscriber("/US3", Range, callbackUltrasonDroit) #callback Ultrason1
    #rospy.Subscriber("camera/depth/image", Type??, callbackBalance) #callback balance
    
    #departBase()    
    global direction, status_robot, pos_obstacle, num_user
    while not rospy.is_shutdown():
        if status_robot == Status.SQUELETTE:
            if pos_obstacle != PosObstacle.AV_GAUCHE and pos_obstacle != PosObstacle.AV_DROITE:
                twist = Twist()
                twist.linear.x = VALEUR_X
                if direction == Direction.NO :
                    twist.angular.z = VALEUR_ANGLE
                elif direction == Direction.NE: 
                    twist.angular.z = -VALEUR_ANGLE
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

