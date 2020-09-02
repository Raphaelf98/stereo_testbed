#!/usr/bin/env python
import rospy
import numpy as np
from numpy.polynomial import polynomial as pl
from matplotlib.pyplot import *
from std_msgs.msg import String
from package1.msg import vectorOfPoints
#from mpl_toolkits.mplot3d.axes3d import Axes3D
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import UnivariateSpline
from scipy.interpolate import BSpline
from scipy import interpolate
import matplotlib.pyplot as plt
import scipy.spatial as ss
from scipy.stats import linregress
from time import time
import math
i=1
millis= 0
tcp_sensivity = 50
sorted_index = []
def callback(msg):
    global i, millis, tcp_sensivity
    global sorted_index
    #condition for online graphing
    #if(i % 4) ==0:
    #single execution for testing
    if(i) < 2:
         milliseconds = int(time() * 1000)
         fps = 1/((milliseconds-millis)/1000)
         millis = milliseconds
         #plt.xlabel('x')
         #plt.ylabel('y')
         #plt.ylim((-0.5,0.5))
         #plt.xlim((0,0.5))
         #plt.title('polynomial fitting', dict(size=15))
         #plt.text(40,400, round(fps) , dict(size=10))
         #plt.text(10,400, 'fps: ', dict(size=10))
         #plt.text(200,1000, 'contour points: ' , dict(size=10))
         #plt.text(310,1000, len(msg.points), dict(size=10))
         #plt.text(600,1000, 'polynomial degree: 7', dict(size=10))
         x_data =[]
         y_data =[]
         z_data =[]
         neighbourhoodDistancethreshold =  0.015 #math.sqrt(2)
         x_data_sorted=[]
         y_data_sorted=[]
         z_data_sorted=[]
         x_data_test = np.array([600,500,600,100,500,600])
         y_data_test = np.array([100,400,600,600,400,100])
         for t in range (0,len(msg.points)):

             x_data.append(msg.points[t].x)
             y_data.append(msg.points[t].y)
             z_data.append(msg.points[t].z)
         #plt.plot(x_vals,y_vals, 'ro' ,markersize=2)
         #if
         #print("x0val: ", x_data[0],"   y0val: ", y_data[0] , " z0val:" , z_data[0], "   size of skel. Array: ", len(msg.points) )
         print("size of data containers: " , "x: ", len(x_data) , "   y: ", len(y_data), "z: ", len(z_data) )
         print("data last index: " , "x: ", x_data[len(x_data)-1] , "   y: ", y_data[len(y_data)-1], "z: ", z_data[len(z_data)-1] )
         #fit polynomial of degree n
         print('x vals',     x_data)

         #data = np.array([x_data,y_data])
         data = np.array([x_data,y_data,z_data])

         #method finds index of lowest x-value
         def startingPoint(values):
             minElementIndex = np.where(values == np.amin(values))
             print('start coordinates: ' ,minElementIndex)
             return minElementIndex[0][0]

         def sort_dots(metrics, start):

             dist_m = ss.distance.squareform(ss.distance.pdist(data.T, metrics))

             total_points = data.shape[1]
             points_index = set(range(total_points))
             sorted_index = []
             target    = start
             #plot(data[0, target], data[1, target], 'o', markersize=16)

             points_index.discard(target)
             while len(points_index)>0:
                 candidate = list(points_index)
                 nneigbour = candidate[dist_m[target, candidate].argmin()]

                 if (dist_m[target, nneigbour] <= neighbourhoodDistancethreshold):

                       sorted_index.append(target)

                       points_index.discard(nneigbour)
                       points_index.discard(target)
                 #print points_index, target, nneigbour
                 #print ('distance: ' ,dist_m[target, nneigbour])

                       target = nneigbour
                 else:

                      break

             print('length sorted indeces: ', len(sorted_index))
             print('sorted_index: ',sorted_index)

             for s in range( 0,len(sorted_index)):

                 x_data_sorted.append(data[0,sorted_index[s]])
                 y_data_sorted.append(data[1,sorted_index[s]])
                 z_data_sorted.append(data[2,sorted_index[s]])

         sort_dots('euclidean',startingPoint(x_data))
         print('zdata:' , z_data_sorted)
         print('xdata:' , x_data_sorted)
         print('ydata:' , y_data_sorted)
         def chordLength():
             length = 0
             chordlength= 0
             chordpoints = np.array([x_data_sorted,y_data_sorted])

             dist_m1 = ss.distance.squareform(ss.distance.pdist(chordpoints.T, 'euclidean'))
             print('lenxdata' , len(x_data_sorted))
             for c in range ( 0 , len(x_data_sorted)-1):
                 chordlength +=  dist_m1[c,c+1]
                 #print('dist : ' ,dist_m1[c,c+1])
                 length = chordlength
                 if ( c == len(x_data_sorted)):
                      break

             return chordlength

         def polynomialFitting(x_vals,y_vals,z_vals):

             n_xy=10
             n_xz=5
             p_xy = np.poly1d(np.polyfit(x_vals, y_vals, n_xy))
             p_xz = np.poly1d(np.polyfit(x_vals, z_vals, n_xz))

             t_xy = np.linspace(min(x_vals), max(x_vals), 100)
             t_xz = np.linspace(min(x_vals), max(x_vals), 100)

             plt.plot(x_vals, z_vals,'o',markersize=2)
             #plt.plot(x_vals, y_vals, 'o', t, p(t), '-')
             #plt.plot(t_xy, p_xy(t), '-', color= "orange")
             plt.plot(t_xz, p_xz(t_xz), '-', color= "orange")
             ##tcp constraint with euclidean norm, takes length of vectorOfPoints array and plugs it into the fitted polynomial function p,
             ##the position of the tcp is expected in neighbourhood of the resulting coordinates. tcp_sensivity is equivalent to radius.
             ##only tcp candidates laying within the radius are accepted
             #
             #epsilon = math.sqrt( (max(x_vals)-msg.tcp.x)**2 + (p(max(x_vals))- msg.tcp.y)**2)
             #print('epsilon: ' , epsilon)
             #if epsilon < tcp_sensivity:
             #   plt.plot(msg.tcp.x, msg.tcp.y, marker='o', markersize=5, color="red")


         def splineFittingParametric(x_vals, y_vals):

             chordlength_ = chordLength()
             print('chordlength: ', chordlength_)
             t =  np.linspace( 0 , chordlength_ , len(x_vals), endpoint=False)

             #x_spline = UnivariateSpline(t, x_vals)
             #y_spline = UnivariateSpline(t, y_vals)
             x_spline = BSpline(t, x_vals,k=2)
             y_spline = BSpline(t, y_vals,k=2)
             t_ = np.arange( 0,chordlength_ , 0.01)
             plt.plot(x_vals, y_vals,'o',markersize=2)
             plt.plot(x_spline(t_), y_spline(t_))


         def polynomialFittingParametric(x_vals, y_vals):

             chordlength_ = chordLength()
             print('chordlength: ', chordlength_)
             t =  np.linspace(0, chordlength_ , len(x_vals), endpoint=True)
             x_params = np.polyfit(t , x_vals , 15)
             y_params = np.polyfit(t , y_vals , 15)
             x = np.poly1d(x_params)
             y = np.poly1d(y_params)
             t_ = np.arange( 0, chordlength_ , 0.01)
             plt.plot(x_vals, y_vals,'o',markersize=2)
             plt.plot( x(t_) , y(t_) )

         def closedFormInterpolatation(x_vals, y_vals):

             x_vals = np.r_[x_vals, x_vals[0]]
             y_vals = np.r_[y_vals, y_vals[0]]
             tck, u = interpolate.splprep([x_vals, y_vals], s=0, per=False)
             xi, yi = interpolate.splev(np.linspace(0, 1, 1000), tck)
             plt.plot(x_vals, y_vals,'o',markersize=2)
             plt.plot( xi , yi )
         def spatialPolynomialFittingParametric(x_vals, y_vals, z_vals):

             plt.rcParams['legend.fontsize'] = 10
             fig = plt.figure()
             ax = fig.gca(projection='3d')

             chordlength_ = chordLength()
             print('chordlength: ', chordlength_)
             t =  np.linspace(0, chordlength_ , len(x_vals), endpoint=True)
             x_params = np.polyfit( t , x_vals , 10)
             y_params = np.polyfit( t , y_vals , 10)
             z_params = np.polyfit( t , z_vals , 10)
             x = np.poly1d(x_params)
             y = np.poly1d(y_params)
             z = np.poly1d(z_params)
             t_ = np.arange( 0, chordlength_ , 0.001)
             ax.plot( x_vals, y_vals , z_vals,'o', markersize=2)
             ax.plot( x(t_) , y(t_), z(t_) )
             plt.show()
         #polynomialFitting(x_data, y_data, z_data)
         #polynomialFittingParametric(x_data_sorted,y_data_sorted)
         #splineFittingParametric(x_data_sorted,y_data_sorted)
         #closedFormInterpolatation(x_data, y_data)
         spatialPolynomialFittingParametric( x_data_sorted, y_data_sorted , z_data_sorted)
         #plt.draw()

         #plt.pause(0.0001)
         #plt.clf()
         x_data.clear()
         y_data.clear()
         z_data.clear()
    else:
         print("pause")
    i+=1






def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('SkeletonPoints', vectorOfPoints, callback, queue_size = 1)
    print("hallo")
    rospy.spin()

plt.show()

listener()

