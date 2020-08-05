#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import pandas as pd
from std_msgs.msg import String
from package1.msg import vectorOfPoints
from matplotlib.animation import FuncAnimation
from array import *
import numpy as np
#def callback(data):
#    positionx0= data.points[0].x
#    positiony0= data.points[0].y
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s",(positionx0,positiony0))
#
#def listener():
#    rospy.init_node('listener', anonymous=True)
#    rospy.Subscriber('SkeletonPoints', vectorOfPoints, callback)
#
#    # spin() simply keeps python from exiting until this node is stopped



def callback(msg):

    plt.xlabel('x')
    plt.ylabel('y')
    plt.ylim((0,1500))
    plt.xlim((0,1500))

    x_vals =[]
    y_vals =[]
    for t in range (0,len(msg.points)):

        x_vals.append(msg.points[t].x)
        y_vals.append(msg.points[t].y)

    #plt.plot(x_vals,y_vals, 'ro' ,markersize=2)
    plt.scatter(x_vals,y_vals)
    plt.draw()

    plt.pause(0.00001)
    plt.clf()

def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('SkeletonPoints', vectorOfPoints, callback)
    print("hallo")
    rospy.spin()

plt.show()
listener()

