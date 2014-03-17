#/usr/bin/env python

#necessaire pour ce programme :
#roscore
#rosrun kinect_aux kinect_aux_node
#rostopic pub /tilt_angle std_msgs/Float64 -- 15

import rospy
from geometry_msgs.msg import Twist
import sys, tty, termios, signal
import numpy as np
import math
from std_msgs.msg import Int32, Float64
from sensor_msgs.msg import Range

if __name__ == '__main__':
    try:
        # On lui fait Baisser la "tete"	
 	rospy.init_node('kinectturtle', log_level=rospy.DEBUG)        
	kinectMotorPub = rospy.Publisher('/tilt_angle', Float64)
        kinectMotorPub.publish(-15)
	rospy.sleep(1)
	kinectMotorPub.publish(15)

    except rospy.ROSInterruptException:
        pass

