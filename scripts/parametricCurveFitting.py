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
import csv
import PySimpleGUI as sg

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
         plt.rcParams['legend.fontsize'] = 10
         fig = plt.figure()
         ax = fig.gca(projection='3d')
         ax.set_xlim(0,0.2)
         ax.set_ylim(0,0.2)
         ax.set_zlim(0,0.2)
         ax.set_xlabel('X-Axis')
         ax.set_ylabel('Y-Axis')
         ax.set_zlabel('z-Axis')
         x_data =[]
         y_data =[]
         z_data =[]
         neighbourhoodDistancethreshold =  0.03 #math.sqrt(2)
         sampleRate = 25
         x_samples =[]
         y_samples =[]
         z_samples =[]
         x_data_sorted=[]
         y_data_sorted=[]
         z_data_sorted=[]
         x_data_test = np.array([600,500,600,100,500,600])
         y_data_test = np.array([100,400,600,600,400,100])
         z_data_test = np.array([100,100,100,200,200,200])

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

         #method finds index of lowest x-values
         def startingPoint(values):
             minElementIndex = np.where(values == np.amin(values))
             print('start coordinates: ' ,minElementIndex)
             return minElementIndex[0][0]


        #https://www.algorithms-and-technologies.com/dijkstra/python
         def Dijkstra(metrics, start):
             Dijkstra_parameter = 0.0015
             adjacency_matrix = ss.distance.squareform(ss.distance.pdist(data.T, metrics))
             adjacency_matrix[adjacency_matrix > Dijkstra_parameter] = 0
             Graph =[]
             print('adjacency_matrix' , adjacency_matrix[0])

             distances = [float("inf") for _ in range(len( adjacency_matrix))]
             print('distances: ' , distances)
             visited = [False for _ in range(len( adjacency_matrix))]
             distances[start] = 0

             while True:

                 shortest_distance = float("inf")
                 shortest_index = -1

                 for i in range(len(adjacency_matrix)):

                     if distances[i] < shortest_distance and not visited[i]:
                         shortest_distance = distances[i]
                         Graph.append(i)
                         shortest_index = i

                 if shortest_index == -1:
                     break


                 for i in range(len(adjacency_matrix[shortest_index])):

                     if adjacency_matrix[shortest_index][i] != 0 and distances[i] > distances[shortest_index] + adjacency_matrix[shortest_index][i]:
                         distances[i] = distances[shortest_index] + adjacency_matrix[shortest_index][i]

                 visited[shortest_index] = True

             for k in range( 0 ,len(Graph)):

                 x_data_sorted.append(data[0,Graph[k]])
                 y_data_sorted.append(data[1,Graph[k]])
                 z_data_sorted.append(data[2,Graph[k]])
             return distances

         def sort_dots(metrics, start):

             dist_m = ss.distance.squareform(ss.distance.pdist(data.T, metrics))
             print('distm' , dist_m[0])
             total_points = data.shape[1]
             points_index = set(range(total_points))
             sorted_index = []
             target    = start
             ax.plot( [data[0, target]] ,  [data[1, target]], [data[2, target]], 'ro')

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

         def curvatureConstraint(x_sor,y_sor, z_sor):
             Curv3DList = []
             posOld_x = 0
             posOld_y = 0
             posOld_z = 0
             posOlder_x=0
             posOlder_y=0
             posOlder_z=0

             for s in range(0, len(x_sor)):
                pos_x = x_sor[s]
                pos_y = y_sor[s]
                pos_z = z_sor[s]
                if (t) == 0 :
                    posOld_x= posOlder_x=pos_x
                    posOld_y= posOlder_y=pos_y
                    posOld_z= posOlder_z=pos_z
                firstDerivative_x = pos_x - posOld_x
                firstDerivative_y = pos_y - posOld_y
                firstDerivative_z = pos_y - posOld_z
                secondDerivative_x = pos_x - 2.0*posOld_x+posOlder_x
                secondDerivative_y = pos_y - 2.0*posOld_y+posOlder_y
                secondDerivative_z = pos_z - 2.0*posOld_z+posOlder_z


                curvature3D = math.sqrt( ( math.pow(secondDerivative_z*firstDerivative_y-secondDerivative_y*firstDerivative_z,2) + math.pow(secondDerivative_x*firstDerivative_z-secondDerivative_z*firstDerivative_x,2) + math.pow(secondDerivative_y*firstDerivative_x-secondDerivative_x*firstDerivative_y,2) ) / math.pow(math.pow(firstDerivative_x,2)+math.pow(firstDerivative_y,2)+math.pow(firstDerivative_z,2),3) )
                Curv3DList.append(curvature3D)

         #sort_dots('euclidean',startingPoint(x_data))
         print('Disjkstra', Dijkstra('euclidean',startingPoint(x_data)))
         #curvatureConstraint(x_data_sorted,y_data_sorted,z_data_sorted)
         #ax.plot([x_data_sorted[ len(x_data_sorted)-1]] , [y_data_sorted[len(y_data_sorted)-1]],[ z_data_sorted[len(z_data_sorted)-1]], 'o')
         print('zdata:' , z_data_sorted)
         print('xdata:' , x_data_sorted)
         print('ydata:' , y_data_sorted)
         def chordLength():
             length = 0
             chordlength= 0
             chordpoints = np.array([x_data_sorted,y_data_sorted,z_data_sorted])

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


         def splineFittingParametric(x_vals, y_vals,z_vals):

             chordlength_ = chordLength()
             #chordlength_ = 0.168
             print('chordlength: ', chordlength_)
             t =  np.linspace( 0 , chordlength_ , len(x_vals), endpoint=False)

             #x_spline = UnivariateSpline(t, x_vals)
             #y_spline = UnivariateSpline(t, y_vals)
             x_spline = BSpline(t, x_vals,k=2)
             y_spline = BSpline(t, y_vals,k=2)
             z_spline = BSpline(t, z_vals,k=2)
             t_ = np.arange( 0,chordlength_ , 0.01)
             ax.plot(x_vals, y_vals,z_vals,'o',markersize=2)

             ax.plot(x_spline(t_), y_spline(t_),z_spline(t_))
             plt.show()

         def polynomialFittingParametric(x_vals, y_vals):

             chordlength_ = chordLength()
             print('chordlength: ', chordlength_)
             t =  np.linspace(0, chordlength_ , len(x_vals), endpoint=True)
             x_params = np.polyfit(t , x_vals , 15)
             y_params = np.polyfit(t , y_vals , 15)
             x = np.poly1d(x_params)
             y = np.poly1d(y_params)
             t_ = np.arange( 0, chordlength_ , 0.001)
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
             t_scl = []



             chordlength_ = chordLength()
             print('chordlength: ', chordlength_)
             t =  np.linspace(0, chordlength_ , len(x_vals), endpoint=True)
             x_params = np.polyfit( t , x_vals , 10)
             y_params = np.polyfit( t , y_vals , 10)
             z_params = np.polyfit( t , z_vals , 10)
             x = np.poly1d(x_params)
             y = np.poly1d(y_params)
             z = np.poly1d(z_params)
             #take samples at defined intervalls
             t_sample = np.arange(0, chordlength_, (chordlength_/sampleRate))
             t_scl = np.append(t_sample, [chordlength_])

             print('smapl', t_sample)
             print('smapl + cl', t_scl)
             for i in range (0, len(t_scl)):
                   x_samples.append( x(t_scl[i]))
                   y_samples.append( y(t_scl[i]))
                   z_samples.append( z(t_scl[i]))

                   print(i,' ','sample', x_samples[i],' ','t: ', t_scl[i])
             #print( 'x_samples' , x_samples[4])
             t_ = np.arange( 0, chordlength_ , 0.001)
             sg.theme('DarkAmber')
             layout = [  [sg.Text('Do you want to save data to CSV file? ')],
                         [sg.Button('Ok'), sg.Button('Cancel')] ]
             window = sg.Window('Test writer', layout)


             ax.plot( x_vals, y_vals , z_vals,'o', markersize=2)
             ax.plot( x(t_) , y(t_), z(t_) )

             plt.show()
             while True:

                 event,values = window.read()

                 if event == sg.WIN_CLOSED or event == 'Cancel':
                     break

                 if event == 'Ok':
                     CSVFileCreator(t_scl,x_samples,y_samples,z_samples)
                     print('Data has been added to CSV file')
                     break
             window.close()
         #polynomialFitting(x_data, y_data, z_data)
         #polynomialFittingParametric(x_data_sorted,y_data_sorted)
         #splineFittingParametric(x_data_sorted,y_data_sorted,z_data_sorted)
         #closedFormInterpolatation(x_data, y_data)
         spatialPolynomialFittingParametric( x_data_sorted, y_data_sorted , z_data_sorted)
         #spatialPolynomialFittingParametric( x_data_test, y_data_test , z_data_test)
         #plt.draw()

         #plt.pause(0.0001)
         #plt.clf()
         x_data.clear()
         y_data.clear()
         z_data.clear()
    else:
         print("pause")
    i+=1

def CSVFileCreator(t,x_smpls, y_smlps, z_smpls):
    rows= [t,x_smpls,y_smlps,z_smpls]
    row = [t]
    with open ('/home/ulzea/RESULTS/DataAnalysis/curveFittingSamples.csv' , 'a+') as file:
         writer = csv.writer(file)
         #writer.writerow(row)
         writer.writerows(rows)
def appendRowToCSV(t,x_smpls, y_smlps, z_smpls):
    rows= [t,x_smpls,y_smlps,z_smpls]
    with open('/home/ulzea/RESULTS/DataAnalysis/curveFittingSamples.csv', 'a+', newline = '' ) as file:
         writer = csv.writer(file)
         #writer.writerow(row)
         writer.writerows(rows)
def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('SkeletonPoints', vectorOfPoints, callback, queue_size = 1)

    print("hallo")
    rospy.spin()

plt.show()

listener()

