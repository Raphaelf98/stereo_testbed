#!/usr/bin/env python
import rospy
import numpy as np
import time
from numpy.polynomial import polynomial as pl
from matplotlib.pyplot import *
from std_msgs.msg import String
from package1.msg import vectorOfPoints
#from mpl_toolkits.mplot3d.axes3d import Axes3D
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import UnivariateSpline
from scipy.interpolate import BSpline
import scipy.interpolate as interpolate
import matplotlib.pyplot as plt
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from scipy.sparse.csgraph import shortest_path
from scipy.integrate import quad
from scipy.optimize import fsolve
from scipy.misc import derivative
import scipy.spatial as ss
from scipy.stats import linregress
from time import time
import math
import csv
import PySimpleGUI as sg
import timeit
import matplotlib
#matplotlib.use("pgf")
#matplotlib.rcParams.update({
#    "pgf.texsystem": "pdflatex",
#    'font.family': 'serif',
#    'text.usetex': True,
#    'pgf.rcfonts': False,
#})
i=1
millis= 0
tcp_sensivity = 50
sorted_index = []
t = 0
jump_once_more = True
collected_distances =[]
def callback(msg):
    global i, millis, tcp_sensivity
    global sorted_index
    global collected_distances

    #condition for online graphing
    #if(i % 4) ==0:
    #single execution for testing
    if(i) < 2:
    #if(True):
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
         #fig1, ax1 = plt.subplots()
         #ax.set_xlim(-0.1,0.1)
         #ax.set_ylim(-0.1,0.1)
         #ax.set_zlim(-0.1,0.1)
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
         flipped_shortest_path = []
         starttime = timeit.default_timer()
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
         #print('x vals',     x_data)

            #data = np.array([x_data,y_data])
         data = np.array([x_data,y_data,z_data])

         #method finds index of lowest x-values
         def startingPoint(values):
             minElementIndex = np.where(values == np.amin(values))
             #minElementIndex = np.where(values == np.amax(values))
             print('start coordinates: ' ,'x: ' ,x_data[minElementIndex[0][0]],' y: ' ,y_data[minElementIndex[0][0]],'z: ' ,z_data[minElementIndex[0][0]])
             return minElementIndex[0][0]


         adjacency_matrix = ss.distance.squareform(ss.distance.pdist(data.T, 'euclidean'))
         Dijkstra_parameter = 0.002
         adjacency_matrix[adjacency_matrix > Dijkstra_parameter] = 0

         visited = [False for _ in range(len( adjacency_matrix))]
         distances = [float("inf") for _ in range(len( adjacency_matrix))]
         #print('len distances: ' ,len(distances),'distances', distances[500])
         already_visited =[]


        #https://www.algorithms-and-technologies.com/dijkstra/python
         def Dijkstra(metrics, start):
             global t
             global jump_once_more
             next_iteration= False



             Jump_threshold = 0.01
             min_jump_threshold =0.002

             adjacency_matrix_full = ss.distance.squareform(ss.distance.pdist(data.T, metrics))
             adjacency_matrix_full[adjacency_matrix_full > Jump_threshold] = 0
             adjacency_matrix_full[adjacency_matrix_full < min_jump_threshold] = 0




             graph =[]
             #print('adjacency_matrix' , adjacency_matrix[0])


             distances[start] = 0

             while True:

                 shortest_distance = float("inf")
                 shortest_index = -1

                 for i in range(len(adjacency_matrix)):

                     if distances[i] < shortest_distance and not visited[i]:
                         shortest_distance = distances[i]
                         shortest_index = i
                 #print('shortest_index ',shortest_index)
                 #print('shortest_distance ',shortest_distance)
                 if shortest_index == -1:
                     break


                 for i in range(len(adjacency_matrix[shortest_index])):


                     if adjacency_matrix[shortest_index][i] != 0 and distances[i] > distances[shortest_index] + adjacency_matrix[shortest_index][i]:
                         distances[i] = distances[shortest_index] + adjacency_matrix[shortest_index][i]
                         graph.append([i,shortest_index])

                         #graph.append([i,shortest_index])
                         #print("Updating distance of node " + str(i) + " to " + str(distances[i]))
                         #parents[i]=shortest_index
                 already_visited.append(shortest_index)
                 visited[shortest_index] = True
             shortest_path =[]
             #print('graph' , graph)
             if len(graph) != 0:
                    shortest_path.append(graph[len(graph)-1][0])
                    pos = graph[len(graph)-1][1]
                    for i in reversed(graph):
                        if i[0] == pos:
                            shortest_path.append(i[0])
                            shortest_path.append(i[1])
                            pos = i[1]

                    shortest_path[::-1]


                    for i in range(0, len(shortest_path)):
                          flipped_shortest_path.append(shortest_path[len(shortest_path)-i-1])

                    #print('path',  flipped_shortest_path)

                    next_position_list = np.array(adjacency_matrix_full[flipped_shortest_path[-1]])

                    #print('position', flipped_shortest_path[-1],'next_position list length  ', len(next_position_list) ,'next_position list ' ,next_position_list)
                    for f in range(0,len(already_visited)):
                         next_position_list[already_visited[f]] = 0.0

                    #print('position', flipped_shortest_path[-1],'next_position list length  ', len(next_position_list) ,'next_position list ' ,next_position_list)
                    #print('len adjacency_matrix', len(adjacency_matrix) , 'len already_visited', len(already_visited))
                    if not np.all(next_position_list==0) and len(already_visited) < len(adjacency_matrix):
                           start_= np.where(next_position_list == np.min(next_position_list[np.nonzero(next_position_list)]))
                           #flipped_shortest_path.append(start_[0][0])
                           print('next start node ', start_[0][0])
                           ax.plot(data[0,start_[0]],data[1,start_[0]],data[2,start_[0]], 'bo', markersize= 6, label='New starting point')
                           jump_once_more= True
                           next_iteration = True
                    #if len(flipped_shortest_path) < len(adjacency_matrix) :

             if len(graph) == 0 and jump_once_more == True:
                    next_position_list = np.array(adjacency_matrix_full[start])
                    for f in range(0,len(already_visited)):
                         next_position_list[already_visited[f]] = 0.0
                    if not np.all(next_position_list==0) and len(already_visited) < len(adjacency_matrix):
                        print('jump once more')
                        start_= np.where(next_position_list == np.min(next_position_list[np.nonzero(next_position_list)]))
                        ax.plot(data[0,start_[0]],data[1,start_[0]],data[2,start_[0]], 'go', markersize= 6)
                        next_iteration = True
                        jump_once_more= False

             collected_distances.append(distances)
             #print( 'colected distances ' , collected_distances)
             print('flipped shortest path', flipped_shortest_path)

             #flipped_shortest_path = list(dict.fromkeys(flipped_shortest_path))
             for k in range( 0 ,len(flipped_shortest_path)):

                 x_data_sorted.append(data[0,flipped_shortest_path[k]])
                 y_data_sorted.append(data[1,flipped_shortest_path[k]])
                 z_data_sorted.append(data[2,flipped_shortest_path[k]])

             #print('len xdarta' ,len(x_data_sorted), 'x: ', x_data_sorted)
             shortest_path.clear()
             print('flipped shortest path', flipped_shortest_path)
             flipped_shortest_path.clear()

             if next_iteration:
                 if start_[0][0] != 0:
                    #print('adjacency_matrix', adjacency_matrix[start_[0][0]])
                    if not np.all(adjacency_matrix[start_[0][0]] == 0):
                        print('next iter starts                                                              ')
                        Dijkstra('euclidean', start_[0][0])

         def get_path(Pr, i, j):
             path = [j]
             k = j
             print('K', k)
             while Pr[k] != -9999:
                 path.append(Pr[k])
                 k = Pr[k]
             path.append(i)
             return path[::-1]

         Dijkstra_parameter1 = 0.005
         def Dijkstra2(start):
             adjacency_matrix1 = ss.distance.squareform(ss.distance.pdist(data.T, 'euclidean'))

             adjacency_matrix1[adjacency_matrix1 > Dijkstra_parameter1] = 0
             Graph = csr_matrix(adjacency_matrix1)
             #standard library version
             dist_matrix, predecessors = shortest_path(csgraph=Graph,method='D'  ,directed=False, indices=start, return_predecessors=True)
             max_element = np.argmax(dist_matrix)
             #print('start', max_element)
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
         #Dijkstra('euclidean',startingPoint(x_data))
         #previous_path = np.array(collected_distances)
         #inf_removed = previous_path[np.isfinite(previous_path)]
         #max_index = max(inf_removed)

         def chordLength():

             chordlength= 0

             chordlength =  max_index

                 #print('dist : ' ,dist_m1[c,c+1])
             if (chordlength) == float("inf"):
                 print("unsufficient amount of z coordinate assignments, Please adjust stereo proc settings in rqt configure or increase Dijkstra threshold!")
                 rospy.signal_shutdown("an exception")
                 time.sleep(500)
             return chordlength


         def splineFittingParametric(x_vals, y_vals,z_vals):

             chordlength_ = chordLength()
             #chordlength_ = 0.168
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

             time1 = timeit.default_timer() - starttime
             print("The time difference is :", time1)
             #CSVTimeWriter(len(adjacency_matrix), time1)
             t_sample = np.linspace(0, chordlength_, sampleRate, endpoint=True)
             #t_scl = np.append(t_sample)
             t_scl=t_sample
             print('smapl', t_sample)
             print('smapl + cl', t_scl)
             for i in range (0, len(t_scl)):
                   x_samples.append( x_spline(t_scl[i]))
                   y_samples.append( y_spline(t_scl[i]))
                   z_samples.append( z_spline(t_scl[i]))

                   print(i,' ','sample', x_samples[i],' ','t: ', t_scl[i])
             #print( 'x_samples' , x_samples[4])
             t_ = np.arange( 0, chordlength_ , 0.0001)
             sg.theme('DarkAmber')
             layout = [  [sg.Text('Do you want to save data to CSV file? ')],
                         [sg.Button('Ok'), sg.Button('Cancel')] ]
             window = sg.Window('Test writer', layout)


             #ax.set_zlim(-0.02,0)
             #ax.set_xlim(0,0.08)
             #ax.set_ylim(-0.03,0.04)

             #ax.set_zlim(-0.05,0.05)
             #ax.set_xlim(0,0.1)
             #ax.set_ylim(0,0.05)

             #ax.set_zlim(-0.05,0.05)
             #ax.set_xlim(0,0.08)
             #ax.set_ylim(0,0.07)
             #ax.set_zlim(-0.05,0.05)
             #ax.set_xlabel('X-Axis')
             #ax.set_ylabel('Y-Axis')
             #ax.set_zlabel('Z-Axis')
             #ax.plot( x_data,y_data,z_data, 'co', markersize= 1,label='Visited nodes')
             ##ax.plot(x_vals, y_vals, z_vals,'--o',markersize=2,label='Shortest path')
             #ax.plot( [ x_vals[0]], [y_vals[0]], [z_vals[0]],'bo',markersize=8, label ='Starting point')
             #ax.plot( [x_vals[ len(x_vals)-1 ]], [y_vals[ len(x_vals)-1 ]], [z_vals[ len(x_vals)-1 ]],'ro',markersize=8,label ='End point')
             #ax.plot(x_spline(t_), y_spline(t_), z_spline(t_),'k',label='3rd degree B-spline' )
             #ax.plot([],[],[],' ',label='DP in [m]: '+str(Dijkstra_parameter1) )
             #ax.plot([],[],[],' ', label='Unsorted size: '+str(len(x_data)) )
             #ax.plot([],[],[],' ', label='Sorted size: '+str(len(x_vals)) )

             #ax.text(0.0, 0.015, -0.015, 'DP: 0.01 [m]', color='black')
             #ax.text(0.0, 0.015, -0.013, 'Path size: '+ str(len(x_vals)), color='black')
             #ax.text(0.0, 0.015, -0.011, 'Sample size: '+ str(len(x_data)), color='black')

             ax.legend(loc="upper left")
             #ax.view_init(-137,-30)
             ax.view_init(-130,60)
             #plt.savefig('/home/ulzea/RESULTS/Plots/3DPGF/worm.pgf')
             ax1.set(xlabel= 'X [m]')
             ax1.set(ylabel= 'Y [m]')
             ax1.plot( x_data,y_data, 'co', markersize= 2,label='Visited nodes')
             ax1.plot(x_vals, y_vals,'--k',markersize=2,label='Shortest path')
             ax1.plot( [ x_vals[0]], [y_vals[0]],'bo',markersize=8, label ='Starting point')
             ax1.plot( [x_vals[ len(x_vals)-1 ]], [y_vals[ len(x_vals)-1 ]], 'ro',markersize=8,label ='End point')
             ax1.legend(loc="upper left")
             ax1.plot([],[],' ',label='DP in [m]: '+str(Dijkstra_parameter1) )
             ax1.plot([],[],' ', label='Unsorted size: '+str(len(x_data)) )
             ax1.plot([],[],' ', label='Sorted size: '+str(len(x_vals)) )
             plt.pause(0.05)
             #ax1.legend(loc="upper right")
             #plt.savefig('/home/ulzea/RESULTS/Plots/3DPGF/loop2D3dbs.pgf')
             #ax1.plot(x_spline(t_), y_spline(t_))


             plt.show()
             while True:

                 event,values = window.read()

                 if event == sg.WIN_CLOSED or event == 'Cancel':
                     break

                 if event == 'Ok':
                     #CSVFileCreator_DATA(x_data,y_data,z_data)
                     #CSVFileCreator(t_scl,x_samples,y_samples,z_samples)
                     print('Data has been added to CSV file')
                     break
             window.close()


         #Necessary for equidistant sampling
         #tp =  np.linspace(0, 1 , len(x_data_sorted), endpoint=True)
         #x_params = np.polyfit( tp , x_data_sorted , 10)
         #y_params = np.polyfit( tp , y_data_sorted , 10)
         #z_params = np.polyfit( tp , z_data_sorted , 10)
         #xnp0 = np.poly1d(x_params)
         #ynp0 = np.poly1d(y_params)
         #znp0 = np.poly1d(z_params)
         #xnp = np.poly1d(xnp0)
         #ynp = np.poly1d(ynp0)
         #znp = np.poly1d(znp0)
         def x(t):
                   return xnp0(t)
         def y(t):
                   return ynp0(t)
         def z(t):
                   return znp0(t)
             #total arc length calculation at interval [min,max]
         def arclength(min,max):
                   solutions = np.array([0,0])
                   dx = lambda t: derivative(x, t, dx=1e-8)
                   dy = lambda t:  derivative(y, t, dx=1e-8)
                   dz = lambda t: derivative(z, t, dx=1e-8)
                   solutions= quad(lambda t: math.sqrt(dx(t)**2+dy(t)**2+dz(t)**2), min , max)

                   return solutions[0]

         def integrand(t):
                   dx = lambda t: derivative(x, t, dx=1e-8)
                   dy = lambda t: derivative(y, t, dx=1e-8)
                   dz = lambda t: derivative(z, t, dx=1e-8)

                   return math.sqrt(dx(t)**2+dy(t)**2+ dz(t)**2)

         def curve_length(t0, S, length,quadtol):
                   integral = quad(S, 0, t0,epsabs=quadtol,epsrel=quadtol)

                   return integral[0] - length
             #numeric solver: for given arc length determine function parameter t
         def solve_t(curve_diff, length,opttol=1.e-15,quadtol=1e-10):
                   return fsolve(curve_length, 0.0, (curve_diff, length,quadtol), xtol = opttol)[0]


         def equidistantSampler(min, max, samples):
                   equidistantSet =[]
                   arcLength = arclength(min,max)
                   print('arclength',arcLength )
                   sampleLength = arcLength /(samples-1)
                   samplePos = 0
                   for i in range(0,samples):
                       equidistantSet.append(round(solve_t(integrand, samplePos,opttol=1e-5,quadtol=1e-3),6))
                       samplePos = samplePos + sampleLength
                   print('Equidistant set: ',equidistantSet)

                   #return np.array(equidistantSet)
                   return equidistantSet

         def EquidistanPolySampling():
                   samples = 25
                   equidistantSet_ = []
                   equidistantSet_ = equidistantSampler(0, 1.00001, samples)
                   print('Equidistant set: ',equidistantSet_)
                   t_scl =[]
                   for i in range (0, len(equidistantSet_)):
                         t_scl.append( equidistantSet_[i])
                         x_samples.append(xnp0(equidistantSet_[i]))
                         y_samples.append(ynp0(equidistantSet_[i])  )
                         z_samples.append(znp0(equidistantSet_[i])  )
                         print(i,' ','sample', x_samples[i], 'tscl: ', t_scl[i])
                   #print( 'x_samples' , x_samples[4])
                   t_ = np.arange( 0,  1 , 0.0001)
                   tf = np.linspace(0,1,samples, endpoint=True)
                   sg.theme('DarkAmber')
                   layout = [  [sg.Text('Do you want to save data to CSV file? ')],
                               [sg.Button('Ok'), sg.Button('Cancel')] ]
                   window = sg.Window('Test writer', layout)

                   ax.set_xlim(0,0.08)
                   ax.set_ylim(-0.03,0.04)
                   ax.set_zlim(-0.05,0.05)
                   #ax.set_xlim(0,0.1)
                   #ax.set_ylim(0,0.05)
                   #ax.set_zlim(-0.05,0.05)
                   #ax.set_xlim(0,0.1)
                   #ax.set_ylim(0,0.06)

                   #ax.set_zlim(-0.020,0)
                   ax.set_xlabel('X-Axis')
                   ax.set_ylabel('Y-Axis')
                   ax.set_zlabel('Z-Axis')
                   ax.plot( x_data,y_data,z_data, 'co', markersize= 1,label='Visited nodes')
                   ax.plot(x_samples, y_samples, z_samples,'ro',label='Equidistant sample')
                   #ax.plot(x_vals, y_vals, z_vals,'--o',markersize=2,label='Shortest path')
                   #ax.plot( [ x_data_sorted[0]], [y_data_sorted[0]], [z_data_sorted[0]],'bo',markersize=8, label ='Starting point')
                   #ax.plot( [x_data_sorted[ len(x_data_sorted)-1 ]], [y_data_sorted[ len(x_data_sorted)-1 ]], [z_data_sorted[ len(x_data_sorted)-1 ]],'ro',markersize=8,label ='End point')
                   ax.plot( xnp0(t_) , ynp0(t_), znp0(t_),'k',label='10th degree polynomial' )
                   #ax.plot( xnp0(tf) , ynp0(tf), znp0(tf),'go',label='10th degree polynomial' )
                   ax.plot([],[],[],' ',label='Sample size: 25' )
                   ax.plot([],[],[],' ',label='DP in [m]: '+str(Dijkstra_parameter1) )
                   ax.plot([],[],[],' ', label='Unsorted size: '+str(len(x_data)) )
                   ax.plot([],[],[],' ', label='Sorted size: '+str(len(x_data_sorted)) )

                   #ax.text(0.0, 0.015, -0.015, 'DP: 0.01 [m]', color='black')
                   #ax.text(0.0, 0.015, -0.013, 'Path size: '+ str(len(x_vals)), color='black')
                   #ax.text(0.0, 0.015, -0.011, 'Sample size: '+ str(len(x_data)), color='black')

                   ax.legend(loc="upper left")
                   #ax.view_init(-130,120)
                   ax.view_init(-130,64)
                   #plt.savefig('/home/ulzea/RESULTS/NewPlots/EquiSpiral.pgf')
                   #ax.text(0.05, 0.05, 0.05, "startpoint: ["+ str(round(x_vals[0],4)) +','+str(round(y_vals[0],4)) +','+ str(round(z_vals[0],4))+']', color='black')
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
             #take samples at defined intervalls

                   t_sample = np.linspace(0, chordlength_, sampleRate, endpoint= True)
                   t_scl = t_sample
                   time1 = timeit.default_timer() - starttime
                   print("The time difference is :", time1)
                   #CSVTimeWriter(len(adjacency_matrix), time1)
                   print('smapl', t_sample)
                   print('smapl + cl', t_scl)
                   for i in range (0, len(t_scl)):
                         x_samples.append( x(t_scl[i]))
                         y_samples.append( y(t_scl[i]))
                         z_samples.append( z(t_scl[i]))

                         print(i,' ','sample', x_samples[i],' ','t: ', t_scl[i])
                   #print( 'x_samples' , x_samples[4])
                   t_ = np.arange( 0, chordlength_ , 0.0001)
                   sg.theme('DarkAmber')
                   layout = [  [sg.Text('Do you want to save data to CSV file? ')],
                               [sg.Button('Ok'), sg.Button('Cancel')] ]
                   window = sg.Window('Test writer', layout)

                   #ax.set_xlim(0,0.08)
                   #ax.set_ylim(-0.03,0.04)
                   #ax.set_zlim(-0.05,0.05)
                   #ax.set_xlim(0,0.1)
                   #ax.set_ylim(0,0.05)
                   #ax.set_zlim(-0.05,0.05)
                   ax.set_xlim(0,0.1)
                   ax.set_ylim(0,0.06)
                   ax.set_zlim(-0.020,0)
                   ax.set_xlabel('X-Axis')
                   ax.set_ylabel('Y-Axis')
                   ax.set_zlabel('Z-Axis')
                   ax.plot( x_data,y_data,z_data, 'co', markersize= 1,label='Visited nodes')
                   #ax.plot(x_vals, y_vals, z_vals,'--o',markersize=2,label='Shortest path')
                   ax.plot( [ x_vals[0]], [y_vals[0]], [z_vals[0]],'bo',markersize=8, label ='Starting point')
                   ax.plot( [x_vals[ len(x_vals)-1 ]], [y_vals[ len(x_vals)-1 ]], [z_vals[ len(x_vals)-1 ]],'ro',markersize=8,label ='End point')
                   ax.plot( x(t_) , y(t_), z(t_),'k',label='10th degree polynomial' )
                   ax.plot([],[],[],' ',label='DP in [m]: '+str(Dijkstra_parameter1) )
                   ax.plot([],[],[],' ', label='Unsorted size: '+str(len(x_data)) )
                   ax.plot([],[],[],' ', label='Sorted size: '+str(len(x_vals)) )

                   #ax.text(0.0, 0.015, -0.015, 'DP: 0.01 [m]', color='black')
                   #ax.text(0.0, 0.015, -0.013, 'Path size: '+ str(len(x_vals)), color='black')
                   #ax.text(0.0, 0.015, -0.011, 'Sample size: '+ str(len(x_data)), color='black')

                   ax.legend(loc="upper left")
                   ax.view_init(-135,20)
                   #plt.savefig('/home/ulzea/RESULTS/Plots/3DPGF/PolySinApp.pgf')
                   #ax.text(0.05, 0.05, 0.05, "startpoint: ["+ str(round(x_vals[0],4)) +','+str(round(y_vals[0],4)) +','+ str(round(z_vals[0],4))+']', color='black')
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
         #EquidistanPolySampling()
         #polynomialFitting(x_data, y_data, z_data)
         #polynomialFittingParametric(x_data_sorted,y_data_sorted)
         splineFittingParametric(x_data_sorted,y_data_sorted,z_data_sorted)
         #closedFormInterpolatation(x_data, y_data)
         #spatialPolynomialFittingParametric( x_data_sorted, y_data_sorted , z_data_sorted)
         #spatialPolynomialFittingParametric( x_data_test, y_data_test , z_data_test)
         #plt.draw()55

         #plt.pause(0.0001)
         #plt.clf()
         x_data.clear()
         y_data.clear()
         z_data.clear()
    else:
         print("pause")
    i+=1
def CSVTimeWriter(sample_size,t):
    rows = [[sample_size],[t]]
    with open ('/home/ulzea/RESULTS/DataAnalysis/DATA/spline_fits/parabola_spline_RT.csv' , 'a+',newline='') as file:
    #with open ('/home/ulzea/RESULTS/DataAnalysis/DATA/Dijkstra_runtime/Dijkstra_rt.csv' , 'a+',newline='') as file:
         writer = csv.writer(file)
         #writer.writerow(row)
         writer.writerows(rows)
def CSVFileCreator(t,x_smpls, y_smlps, z_smpls):
    rows= [t,x_smpls,y_smlps,z_smpls]
    row = [t]
    with open ('/home/ulzea/RESULTS/DataAnalysis/DATA/poly_fits/poly_fit_sin.csv' , 'a+') as file:
         writer = csv.writer(file)
         #writer.writerow(row)
         writer.writerows(rows)
def CSVFileCreator_DATA(x_smpls, y_smlps, z_smpls):
    rows= [x_smpls,y_smlps,z_smpls]

    with open ('/home/ulzea/RESULTS/DataAnalysis/DATA/Dijksrtraprecision/dataTuples.csv' , 'a+') as file:
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

