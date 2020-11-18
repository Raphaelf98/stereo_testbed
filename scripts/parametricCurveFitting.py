#!/usr/bin/env python
import rospy
import numpy as np
from numpy.polynomial import polynomial as pl
from matplotlib.pyplot import *
from std_msgs.msg import String
from package1.msg import vectorOfPoints
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import UnivariateSpline
from scipy.interpolate import BSpline
import scipy.interpolate as interpolate
import matplotlib.pyplot as plt
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from scipy.sparse.csgraph import shortest_path
from scipy.misc import derivative
import scipy.spatial as ss
import math
import matplotlib

i=1
def callback(msg):
    global i
    #condition for single exection of sorting and approximation
    if(i) < 2:

         plt.rcParams['legend.fontsize'] = 10
         fig = plt.figure()
         ax = fig.gca(projection='3d')

         ax.set_xlabel('X-Axis')
         ax.set_ylabel('Y-Axis')
         ax.set_zlabel('z-Axis')
         x_data =[]
         y_data =[]
         z_data =[]
         x_data_sorted=[]
         y_data_sorted=[]
         z_data_sorted=[]

         #message coordinates are stored in x_data, y_data and z_data.
         for t in range (0,len(msg.points)):

             x_data.append(msg.points[t].x)
             y_data.append(msg.points[t].y)
             z_data.append(msg.points[t].z)

         print("size of data containers: " , "x: ", len(x_data) , "   y: ", len(y_data), "z: ", len(z_data) )
         print("data last index: " , "x: ", x_data[len(x_data)-1] , "   y: ", y_data[len(y_data)-1], "z: ", z_data[len(z_data)-1] )

         data = np.array([x_data,y_data,z_data])

         #method finds index of lowest x-values
         def startingPoint(values):
             minElementIndex = np.where(values == np.amin(values))
             print('start coordinates: ' ,'x: ' ,x_data[minElementIndex[0][0]],' y: ' ,y_data[minElementIndex[0][0]],'z: ' ,z_data[minElementIndex[0][0]])
             return minElementIndex[0][0]

         #adjaceny matrix graph representation with euclidean norm
         adjacency_matrix = ss.distance.squareform(ss.distance.pdist(data.T, 'euclidean'))
         #parameter to set maximum distance between nodes
         Dijkstra_parameter = 0.002
         adjacency_matrix[adjacency_matrix > Dijkstra_parameter] = 0

         visited = [False for _ in range(len( adjacency_matrix))]
         distances = [float("inf") for _ in range(len( adjacency_matrix))]

         already_visited =[]

         # retrive shortest path from predecessors list
         def get_path(Pr, i, j):
             path = [j]
             k = j
             print('K', k)
             while Pr[k] != -9999:
                 path.append(Pr[k])
                 k = Pr[k]
             path.append(i)
             return path[::-1]
         #parameter to set maximum distance between nodes
         Dijkstra_parameter1 = 0.005
         def Dijkstra2(start):

             adjacency_matrix1 = ss.distance.squareform(ss.distance.pdist(data.T, 'euclidean'))
             adjacency_matrix1[adjacency_matrix1 > Dijkstra_parameter1] = 0
             Graph = csr_matrix(adjacency_matrix1)
             #Dijkstra algorithm implementation from Scipy
             dist_matrix, predecessors = shortest_path(csgraph=Graph,method='D'  ,directed=False, indices=start, return_predecessors=True)
             max_element = np.argmax(dist_matrix)

             print('predecessors', predecessors)
             print('dist matrix ' , dist_matrix)
             shortest_path_to_distal_point = get_path(predecessors, start, max_element)
             print('shortest path: ', shortest_path_to_distal_point)
             for k in range( 0 ,len(shortest_path_to_distal_point)):

                 x_data_sorted.append(data[0,shortest_path_to_distal_point[k]])
                 y_data_sorted.append(data[1,shortest_path_to_distal_point[k]])
                 z_data_sorted.append(data[2,shortest_path_to_distal_point[k]])


             return max(dist_matrix[np.isfinite(dist_matrix)])

         max_index = Dijkstra2(startingPoint(x_data))


         def chordLength():

             chordlength= 0

             chordlength =  max_index


             if (chordlength) == float("inf"):
                 print("unsufficient amount of z coordinate assignments, Please adjust stereo proc settings in rqt configure or increase Dijkstra threshold!")
                 rospy.signal_shutdown("an exception")
                 time.sleep(500)
             return chordlength

         #approximates sorted backbone points with third degree B-spline
         def splineFittingParametric(x_vals, y_vals,z_vals):

             chordlength_ = chordLength()

             print('chordlength: ', chordlength_)
             t =  np.linspace( 0 , chordlength_ , len(x_vals), endpoint=True)
             #find b spline representatiokn for 1D curve
             tB_x, c_x,k_x = interpolate.splrep(t,x_vals, s=0,k=3)
             x_spline = BSpline(tB_x, c_x, k_x, extrapolate = True)
             tB_y, c_y,k_y = interpolate.splrep(t,y_vals, s=0,k=3)
             y_spline = BSpline(tB_y, c_y,k_y)
             tB_z, c_z,k_z = interpolate.splrep(t,z_vals, s=0,k=3)
             z_spline = BSpline(tB_z, c_z,k_z)
             t_ = np.arange( 0 , chordlength_ , 0.001)
             ax.set_xlim(0,0.08)
             ax.set_ylim(0,0.07)
             ax.set_zlim(-0.05,0.05)
             ax.set_xlabel('X-Axis')
             ax.set_ylabel('Y-Axis')
             ax.set_zlabel('Z-Axis')
             ax.plot( x_data,y_data,z_data, 'co', markersize= 1,label='Visited nodes')
             ax.plot(x_vals, y_vals, z_vals,'--o',markersize=2,label='Shortest path')
             ax.plot( [ x_vals[0]], [y_vals[0]], [z_vals[0]],'bo',markersize=8, label ='Starting point')
             ax.plot( [x_vals[ len(x_vals)-1 ]], [y_vals[ len(x_vals)-1 ]], [z_vals[ len(x_vals)-1 ]],'ro',markersize=8,label ='End point')
             ax.plot(x_spline(t_), y_spline(t_), z_spline(t_),'k',label='3rd degree B-spline' )
             ax.plot([],[],[],' ',label='DP in [m]: '+str(Dijkstra_parameter1) )
             ax.plot([],[],[],' ', label='Unsorted size: '+str(len(x_data)) )
             ax.plot([],[],[],' ', label='Sorted size: '+str(len(x_vals)) )
             plt.show()

         def spatialPolynomialFittingParametric(x_vals, y_vals, z_vals):

             t_scl = []
             chordlength_ = chordLength()
             #chordlength_ = 1
             print('chordlength: ', chordlength_)
             t =  np.linspace(0, chordlength_ , len(x_vals), endpoint=True)
             x_params = np.polyfit( t , x_vals , 5)
             y_params = np.polyfit( t , y_vals , 5)
             z_params = np.polyfit( t , z_vals , 5)
             x = np.poly1d(x_params)
             y = np.poly1d(y_params)
             z = np.poly1d(z_params)

             t_ = np.arange( 0, chordlength_ , 0.0001)

             ax.set_xlim(0,0.1)
             ax.set_ylim(0,0.06)
             ax.set_zlim(-0.020,0)
             ax.set_xlabel('X-Axis')
             ax.set_ylabel('Y-Axis')
             ax.set_zlabel('Z-Axis')
             ax.plot( x_data,y_data,z_data, 'co', markersize= 1,label='Visited nodes')
             ax.plot(x_vals, y_vals, z_vals,'--o',markersize=2,label='Shortest path')
             ax.plot( [ x_vals[0]], [y_vals[0]], [z_vals[0]],'bo',markersize=8, label ='Starting point')
             ax.plot( [x_vals[ len(x_vals)-1 ]], [y_vals[ len(x_vals)-1 ]], [z_vals[ len(x_vals)-1 ]],'ro',markersize=8,label ='End point')
             ax.plot( x(t_) , y(t_), z(t_),'k',label='10th degree polynomial' )
             ax.plot([],[],[],' ',label='DP in [m]: '+str(Dijkstra_parameter1) )
             ax.plot([],[],[],' ', label='Unsorted size: '+str(len(x_data)) )
             ax.plot([],[],[],' ', label='Sorted size: '+str(len(x_vals)) )

             ax.legend(loc="upper left")
             ax.view_init(-135,20)

             plt.show()



         #function calls for either B-spline approximation or polynomial fitting

         splineFittingParametric(x_data_sorted,y_data_sorted,z_data_sorted)
         #spatialPolynomialFittingParametric( x_data_sorted, y_data_sorted , z_data_sorted)


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

