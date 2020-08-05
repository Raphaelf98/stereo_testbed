#!/usr/bin/env python
# license removed for brevity
import rospy
import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import String
import numpy as np
import math
def grapher():
    distance_Z = np.arange(0.,0.5, 0.01)

    Base_Width = 0.1
    Focal_Length = 1763   #in [pixel]
    Sigma_XY = 0.3   #deviation of measurement in [pixel]
    Sigma_X_Parallax = Sigma_XY*math.sqrt(2) #X parallax standard deviation in [pixel]
    plt.xlabel('Z [m]')
    plt.ylabel('Sigma_Z [m]')
    plt.plot(distance_Z,(distance_Z*distance_Z)/(Focal_Length*Base_Width), 'ro' ,markersize=2)
    plt.show()

if __name__ == "__main__":
    rospy.init_node('grapher', anonymous=True)
    grapher()
