#/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, tty, termios, signal
import numpy as np
import math
from std_msgs.msg import Int32, Float64
from sensor_msgs.msg import Range

#CONSTANTE
VALEUR_X = 0.2
VALEUR_ANGLE = 0.5
TIMEOUT_USER = 20
TIMEOUT_SERVEUR = 5
ULTRASON_MAX_DIST = 100
ULTRASON_ERR = 20
SEUIL_ARRET = 60
SEUIL_OBSTACLE = 30
BALANCE_ERR = 1

#ENUM
class Status:
    NORMAL = 0
    RETOUR = 1
    SQUELETTE = 2
    SERVEUR = 3

class Direction : 
    NORD = 0
    EST = 1
    OUEST = 2
    NE = 3
    NO = 4

#CLASS
class CurrentUltrasoundMesurement :
    timestamp = 0
    ultrasonAvant = 0
    ultrasonGauche = 0
    ultrasonDroit = 0

class PosObstacle:
    avant = False
    av_gauche = False
    av_droite = False
    cote_gauche = False
    cote_droit = False
    arriere = False

#GLOBAL
status_robot = Status.NORMAL
pos_obstacle = PosObstacle()
num_user = 0
timer_squelette = 0
timer_serveur = 0
direction = Direction.NORD
currentUs = CurrentUltrasoundMesurement()

#FUNCTION
#Utils
def stopFollowingTimeOut(event):
     global timer_squelette, status_robot, num_user
     status_robot = Status.NORMAL
     num_user = 0
     rospy.logdebug("T O FOLLOWING USER")
     timer_squelette.shutdown()

def stopServTimeOut(event):
    global status_robot
    status_robot = Status.NORMAL
    rospy.logdebug("NORMAL")
    timer_serveur.shutdown()
    # On lui fait Baisser la "tete"	
    kinectMotorPub = rospy.Publisher('/tilt_angle', Float64)
    kinectMotorPub.publish(-15)

def obstacle():
    return pos_obstacle.avant or pos_obstacle.av_gauche or pos_obstacle.av_droite

def twistDirection():
    twist = Twist()
    twist.linear.x = 0.1
    if direction == Direction.NORD:
        twist.linear.x = VALEUR_X
    elif direction == Direction.EST:
        twist.angular.z = -(VALEUR_ANGLE*2)
    elif direction == Direction.OUEST:
        twist.angular.z = (VALEUR_ANGLE*2)
    elif direction == Direction.NE:
        twist.angular.z = -VALEUR_ANGLE
    elif direction == Direction.NO:
        twist.angular.z = VALEUR_ANGLE

    return twist
        

def majDirection() :
    global direction, currentUs, status_robot
    if currentUs.ultrasonAvant <= SEUIL_ARRET:
        status_robot = Status.SERVEUR
        rospy.logdebug("SERVEUR")
        timer_serveur = rospy.Timer(rospy.Duration(TIMEOUT_SERVEUR), stopServTimeOut)
	# On lui fait lever la "tete"
	kinectMotorPub = rospy.Publisher('/tilt_angle', Float64)
	kinectMotorPub.publish(15)
    elif currentUs.ultrasonGauche <= ULTRASON_MAX_DIST or currentUs.ultrasonDroit <= ULTRASON_MAX_DIST or currentUs.ultrasonAvant <= ULTRASON_MAX_DIST :
        if currentUs.ultrasonGauche > currentUs.ultrasonAvant - ULTRASON_ERR and currentUs.ultrasonGauche < currentUs.ultrasonAvant + ULTRASON_ERR :
	    direction = Direction.NO
        elif currentUs.ultrasonDroit > currentUs.ultrasonAvant - ULTRASON_ERR and currentUs.ultrasonDroit < currentUs.ultrasonAvant + ULTRASON_ERR :
	    direction = Direction.NE
    else :
        direction = Direction.NORD
    #rospy.logdebug("%d", direction)

def eviterObstacle():
    global pos_obstacle
    twist = Twist()
    if pos_obstacle.av_gauche and not pos_obstacle.av_droit:
        #twist.linear.x = 0.1
        twist.angular.z = -VALEUR_ANGLE
    elif pos_obstacle.av_droit and not pos_obstacle.av_gauche:
        #twist.linear.x = 0.1
        twist.angular.z = VALEUR_ANGLE
    elif pos_obstacle.av_gauche and pos_obstacle.av_droit:
        #twist.linear.x = 0.1
        if not pos_obstacle.cote_gauche:
            twist.angular.z = VALEUR_ANGLE*2
        elif not pos_obstacle.cote_droit:
            twist.angular.z = -(VALEUR_ANGLE*2)
    else:
        twist.angular.z = 4.0

    return twist    

#Fonctions callback
def callbackDepartBase(msg):
    global status_robot
    status_robot = Status.NORMAL

def callbackKinectNewUser(msg):
    global num_user, timer_squelette, status_robot
    if status_robot == Status.NORMAL:
        status_robot = Status.SQUELETTE
        rospy.logdebug("SQUELETTE")
        num_user = msg.data
        timer_squelette = rospy.Timer(rospy.Duration(TIMEOUT_USER), stopFollowingTimeOut)

def callbackKinectDeleteUser(msg):
    global status_robot, num_user
    if status_robot == Status.SQUELETTE and msg.data == num_user:
        status_robot = Status.NORMAL
        rospy.logdebug("NORMAL")
        num_user = 0
    	
def callbackBalance(msg):
    global status_robot
    if msg.range >= 0 and msg.range <= BALANCE_ERR:
        status_robot = Status.RETOUR

#Fonctions callback IR
def callbackIRAvant(msg):
    global pos_obstacle
    if msg.range <= SEUIL_OBSTACLE:
        pos_obstacle.avant = True
    else:
        pos_obstacle.avant = False

def callbackIRAvantGauche(msg):
    global pos_obstacle
    if msg.range <= SEUIL_OBSTACLE:
        pos_obstacle.av_gauche = True
    else:
        pos_obstacle.av_gauche = False

def callbackIRAvantDroit(msg):
    global pos_obstacle
    if msg.range <= SEUIL_OBSTACLE:
        pos_obstacle.av_droit = True
    else:
        pos_obstacle.av_droit = False

def callbackIRCoteGauche(msg):
    global pos_obstacle
    if msg.range <= SEUIL_OBSTACLE:
        pos_obstacle.cote_gauche = True
    else:
        pos_obstacle.cote_gauche = False

def callbackIRCoteDroit(msg):
    global pos_obstacle
    if msg.range <= SEUIL_OBSTACLE:
        pos_obstacle.cote_droit = True
    else:
        pos_obstacle.cote_droit = False

def callbackIRArriere(msg):
    global pos_obstacle
    if msg.range <= SEUIL_OBSTACLE:
        pos_obstacle.arriere = True
    else:
        pos_obstacle.arriere = False
        
#Fonctions callback ultrason
def callbackUltrasonAvant(msg):
    global status_robot, direction, currentUs
    if status_robot == Status.SQUELETTE :
        if msg.header.stamp.secs >  currentUs.timestamp :
	    currentUs.timestamp = msg.header.stamp.secs
            currentUs.ultrasonAvant = msg.range
            #rospy.logdebug("MAJ seq : %d US avant : %d", msg.header.stamp.secs, msg.range)
        elif msg.header.stamp.secs ==  currentUs.timestamp :
            currentUs.ultrasonAvant = msg.range
            #rospy.logdebug("new UsAvant : %d", msg.range)
            majDirection()
        #else :
            #rospy.logdebug("outdated usavant")

def callbackUltrasonGauche(msg):
    global status_robot, direction, currentUs
    if status_robot == Status.SQUELETTE :
        if  msg.header.stamp.secs >  currentUs.timestamp :
	    currentUs.timestamp = msg.header.stamp.secs
            currentUs.ultrasonGauche = msg.range
            #rospy.logdebug("MAJ seq : %d US Gauche : %d", msg.header.stamp.secs, msg.range)
        elif msg.header.stamp.secs ==  currentUs.timestamp :
            currentUs.ultrasonGauche = msg.range
            #rospy.logdebug("new UsGauche : %d", msg.range)
            majDirection()
        #else : 
            #rospy.logdebug("outdated usgauche")

def callbackUltrasonDroit(msg):
    global status_robot, direction, currentUs
    if status_robot == Status.SQUELETTE :
        if msg.header.stamp.secs >  currentUs.timestamp :
	    currentUs.timestamp = msg.header.stamp.secs
            currentUs.ultrasonDroit = msg.range
            #rospy.logdebug("MAJ seq : %d US Droit : %d", msg.header.stamp.secs, msg.range)
        elif msg.header.seq ==  currentUs.timestamp :
            currentUs.ultrasonDroit = msg.range
            #rospy.logdebug("new UsDroit : %d", msg.range)
            majDirection()
        #else :
            #rospy.logdebug("outdated usDroit")

def navigationturtle():
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)     
    rospy.init_node('navigationturtle', log_level=rospy.DEBUG)

    #Inscription topic kinect
    rospy.Subscriber("/isUserVisible", Int32, callbackKinectNewUser) #callback kinect squelette detection
    rospy.Subscriber("/userDisapeared", Int32, callbackKinectDeleteUser) #callback kinect squelette detection
    
    #Inscription topic ultrason
    rospy.Subscriber("/US2", Range, callbackUltrasonAvant) #callback UltrasonAvant
    rospy.Subscriber("/US3", Range, callbackUltrasonGauche) #callback UltrasonGauche
    rospy.Subscriber("/US1", Range, callbackUltrasonDroit) #callback UltrasonDroit
    
    #Inscription topic infrarouge
    rospy.Subscriber("/IR3", Range, callbackIRAvant) #callback IR avant
    rospy.Subscriber("/IR4", Range, callbackIRAvantGauche) #callback IR avant gauche
    rospy.Subscriber("/IR2", Range, callbackIRAvantDroit) #callback IR avant droit
    rospy.Subscriber("/IR5", Range, callbackIRCoteGauche) #callback IR cote gauche
    rospy.Subscriber("/IR1", Range, callbackIRCoteDroit) #callback IR cote droit
    rospy.Subscriber("/IR6", Range, callbackIRArriere) #callback IR arriere
    
    #Inscription topic bouton depart
    #rospy.Subscriber("/DepartBase", Range, callbackDepartBase)

    #Inscription topic balance
    #rospy.Subscriber("/Balance", Range, callbackBalance) #callback balance
    
    #departBase()    
    global direction, status_robot, pos_obstacle, num_user
    while not rospy.is_shutdown():
        if obstacle():
            rospy.logdebug("Obstacle")
            #twist = eviterObstacle()
            #pub.publish(twist)
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            pub.publish(twist)
        else:
	    if status_robot == Status.SQUELETTE:
                #rospy.logdebug("Squelette")
                twist = Twist()
		twist.linear.x = VALEUR_X
		if direction == Direction.NO :
		    twist.angular.z = VALEUR_ANGLE
		elif direction == Direction.NE: 
		    twist.angular.z = -VALEUR_ANGLE
                pub.publish(twist)
            elif status_robot == Status.NORMAL:
                pub.publish(twistDirection())    
		#elif status_robot == Status.RETOUR:
		    #code retour base

        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        navigationturtle()
    except rospy.ROSInterruptException:
        pass

