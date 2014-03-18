#/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, tty, termios, signal
import numpy as np
import math
from std_msgs.msg import Int32, Float64
from sensor_msgs.msg import Range
from kobuki_msgs.msg import Sound

#CONSTANTE
VALEUR_X = 0.2
VALEUR_ANGLE = 0.5
TIMEOUT_USER = 20
TIMEOUT_SERVEUR = 10
ULTRASON_MAX_DIST = 150
ULTRASON_ERR = 20
SEUIL_ARRET = 50
SEUIL_OBSTACLE = 39
SEUIL_RETOUR = 31
VARIATION_BALANCE = 20
SEUIL_CRITIQUE_OBSTACLE = 15
ROSPY_SLEEP = 0.1

#ENUM
class Status:
    NORMAL = 0
    RETOUR = 1
    SQUELETTE = 2
    SERVEUR = 3
    BASE = 4

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
status_robot = Status.SQUELETTE
pos_obstacle = PosObstacle()
num_user = 0
timer_squelette = 0
timer_serveur = 0
direction = Direction.NORD
arret_urgence_avant = False
arret_urgence_avant_gauche = False
arret_urgence_avant_droit = False
currentUs = CurrentUltrasoundMesurement()
poid_balance = 0
bipbip = rospy.Publisher("/mobile_base/commands/sound", Sound)
kinectMotorPub = rospy.Publisher('/tilt_angle', Float64)
pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)

#FUNCTION
#Utils
def stopFollowingTimeOut(event):
     global timer_squelette, status_robot, num_user
     status_robot = Status.NORMAL
     num_user = 0
     rospy.logdebug("T O FOLLOWING USER")
     timer_squelette.shutdown()

def stopServ():
    global status_robot, poid_balance, kinectMotorPub
    rospy.logdebug("NORMAL/STOP SERV")
    timer_serveur.shutdown()
    # On lui fait Baisser la "tete"	
    kinectMotorPub.publish(0)
    foundDirectionFree()
    status_robot = Status.NORMAL
    poid_balance = 0

def stopServTimeOut(event):
    stopServ()

def foundDirectionFree():
    global pub
    twist = Twist()
    if currentUs.ultrasonGauche > currentUs.ultrasonDroit:
        twist.angular.z = 2.0
    else:
        twist.angular.z = -2.0

    pub.publish(twist)

    rospy.sleep(ROSPY_SLEEP)

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
    global direction, currentUs, status_robot, timer_serveur, bipbip, kinectMotorPub
    if currentUs.ultrasonAvant <= SEUIL_ARRET:
        status_robot = Status.SERVEUR
        rospy.logdebug("SERVEUR")
        bipbip.publish(Sound(6))
        if timer_squelette != 0:
            timer_squelette.shutdown()
        
        if timer_serveur != 0:
            timer_serveur.shutdown()

        timer_serveur = rospy.Timer(rospy.Duration(TIMEOUT_SERVEUR), stopServTimeOut)

	# On lui fait lever la "tete"
	kinectMotorPub.publish(15)
    elif currentUs.ultrasonGauche <= ULTRASON_MAX_DIST or currentUs.ultrasonDroit <= ULTRASON_MAX_DIST or currentUs.ultrasonAvant <= ULTRASON_MAX_DIST :
        if currentUs.ultrasonGauche > currentUs.ultrasonAvant - ULTRASON_ERR and currentUs.ultrasonGauche < currentUs.ultrasonAvant + ULTRASON_ERR :
	    direction = Direction.NO
        elif currentUs.ultrasonDroit > currentUs.ultrasonAvant - ULTRASON_ERR and currentUs.ultrasonDroit < currentUs.ultrasonAvant + ULTRASON_ERR :
	    direction = Direction.NE
    else :
        direction = Direction.NORD

def eviterObstacle():
    rospy.logdebug("EVITER OBSTACLE")
    global pos_obstacle
    twist = Twist()
    if pos_obstacle.av_gauche and not pos_obstacle.av_droit:
        twist.angular.z = -VALEUR_ANGLE
    elif pos_obstacle.av_droit and not pos_obstacle.av_gauche:
        rospy.logdebug("GO GAUCHE")
        twist.angular.z = VALEUR_ANGLE
    elif pos_obstacle.av_gauche and pos_obstacle.av_droit:
        if not pos_obstacle.cote_gauche:
            twist.angular.z = VALEUR_ANGLE*2
        elif not pos_obstacle.cote_droit:
            twist.angular.z = -VALEUR_ANGLE*2
    elif not pos_obstacle.av_gauche and not pos_obstacle.av_droit and pos_obstacle.avant:
        if not pos_obstacle.cote_gauche:
            twist.angular.z = VALEUR_ANGLE
        else:
            twist.angular.z = -VALEUR_ANGLE

    return twist    

def departBase():
    global pub
    
    twist = Twist()
    twist.linear.x = -VALEUR_X
    pub.publish(twist)

    rospy.sleep(ROSPY_SLEEP)

    twist = Twist()
    twist.angular.z = 3.0
    pub.publish(twist)

    rospy.sleep(ROSPY_SLEEP)

#Fonctions callback
def callbackBouton(msg):
    if msg.range == 1:
        global status_robot
        if status_robot == Status.BASE:
            status_robot = Status.NORMAL
            departBase()
        else:
            rospy.signal_shutdown(0)

def callbackKinectNewUser(msg):
    global num_user, timer_squelette, status_robot
    if status_robot == Status.NORMAL:
        status_robot = Status.SQUELETTE
        rospy.logdebug("SQUELETTE")
        num_user = msg.data
        if timer_squelette != 0:
            timer_squelette.shutdown()
        timer_squelette = rospy.Timer(rospy.Duration(TIMEOUT_USER), stopFollowingTimeOut)

def callbackKinectDeleteUser(msg):
    global status_robot, num_user
    if status_robot == Status.SQUELETTE and msg.data == num_user:
        status_robot = Status.NORMAL
        rospy.logdebug("NORMAL")
        num_user = 0
    	
def callbackBalance(msg):
    global status_robot, poid_balance
    if msg.range <= SEUIL_RETOUR:
        rospy.logdebug("RETOUR")
        status_robot = Status.RETOUR
    else:
        if status_robot == Status.SERVEUR and poid_balance == 0:
            poid_balance = msg.range
        elif status_robot == Status.SERVEUR and poid_balance != 0 and msg.range <= poid_balance - VARIATION_BALANCE:
            rospy.logdebug("VARIATION BALANCE")
            stopServ()

#Fonctions callback IR
def callbackIRAvant(msg):
    global pos_obstacle, arret_urgence_avant
    if msg.range < SEUIL_CRITIQUE_OBSTACLE:
        rospy.logdebug("SHUTDOWN IR AVANT avec %d", msg.range)
        arret_urgence_avant = True
    elif msg.range <= SEUIL_OBSTACLE:
        arret_urgence_avant = False
        pos_obstacle.avant = True
    else:
        arret_urgence_avant = False
        pos_obstacle.avant = False

def callbackIRAvantGauche(msg):
    global pos_obstacle, arret_urgence_avant_gauche
    if msg.range < SEUIL_CRITIQUE_OBSTACLE:
        rospy.logdebug("SHUTDOWN IR AVANT GAUCHE avec %d", msg.range)
        arret_urgence_avant_gauche = True
    elif msg.range <= SEUIL_OBSTACLE:
        arret_urgence_avant_gauche = False
        pos_obstacle.av_gauche = True
    else:
        arret_urgence_avant_gauche = False
        pos_obstacle.av_gauche = False

def callbackIRAvantDroit(msg):
    global pos_obstacle, arret_urgence_avant_droit
    if msg.range < SEUIL_CRITIQUE_OBSTACLE:
        rospy.logdebug("SHUTDOWN IR AVANT DROIT avec %d", msg.range)
        arret_urgence_avant_droit = True
    elif msg.range <= SEUIL_OBSTACLE:
        arret_urgence_avant_droit = False
        pos_obstacle.av_droit = True
    else:
        arret_urgence_avant_droit = False
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
        elif msg.header.stamp.secs ==  currentUs.timestamp :
            currentUs.ultrasonAvant = msg.range
            majDirection()
       
def callbackUltrasonGauche(msg):
    global status_robot, direction, currentUs
    if status_robot == Status.SQUELETTE :
        if  msg.header.stamp.secs >  currentUs.timestamp :
	    currentUs.timestamp = msg.header.stamp.secs
            currentUs.ultrasonGauche = msg.range
        elif msg.header.stamp.secs ==  currentUs.timestamp :
            currentUs.ultrasonGauche = msg.range
            majDirection()

def callbackUltrasonDroit(msg):
    global status_robot, direction, currentUs
    if status_robot == Status.SQUELETTE :
        if msg.header.stamp.secs >  currentUs.timestamp :
	    currentUs.timestamp = msg.header.stamp.secs
            currentUs.ultrasonDroit = msg.range
        elif msg.header.seq ==  currentUs.timestamp :
            currentUs.ultrasonDroit = msg.range
            majDirection()

def navigationturtle():
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
    
    #Inscription topic bouton (depart ou arret)
    rospy.Subscriber("/bouton", Range, callbackBouton)

    #Inscription topic balance
    rospy.Subscriber("/masse", Range, callbackBalance) #callback balance
    
    #departBase()

    global direction, status_robot, pos_obstacle, num_user, pub, arret_urgence_avant, arret_urgence_avant_gauche, arret_urgence_avant_droit
    while not rospy.is_shutdown():
        if not arret_urgence_avant and not arret_urgence_avant_gauche and not arret_urgence_avant_droit:
            if obstacle():
                rospy.logdebug("Obstacle")
                twist = eviterObstacle()
                pub.publish(twist)
            else:
	        if status_robot == Status.SQUELETTE:
                    twist = Twist()
		    if direction == Direction.NO :
		        twist.angular.z = -VALEUR_ANGLE
		    elif direction == Direction.NE: 
		        twist.angular.z = VALEUR_ANGLE
                    elif direction == Direction.NORD:
                        twist.linear.x = VALEUR_X
                    pub.publish(twist)
                elif status_robot == Status.NORMAL:
                    pub.publish(twistDirection())    
	        #elif status_robot == Status.RETOUR:
		    #code retour base
        else:
            twist = Twist()
            twist.angular.x = 0.0

            pub.publish(twist)

        rospy.sleep(ROSPY_SLEEP)

if __name__ == '__main__':
    try:
        navigationturtle()
    except rospy.ROSInterruptException:
        pass

