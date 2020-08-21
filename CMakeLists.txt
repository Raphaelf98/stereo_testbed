cmake_minimum_required(VERSION 3.0.2)
project(package1)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  rviz_visual_tools
  message_generation
    cmake_modules
)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
add_message_files(
    FILES
    vectorOfPoints.msg
    cameraTransformation.msg
    )

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES package1
 CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs message_runtime rviz_visual_tools
 DEPENDS system_lib Eigen
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} )


## The recommended prefix ensures that target names across packages don't collide
add_executable(camera_pub src/camera_pub.cpp)
add_dependencies(camera_pub ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
 target_link_libraries(camera_pub ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

 add_executable(camera_sub src/camera_sub.cpp)
 add_dependencies(camera_sub ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
  target_link_libraries(camera_sub ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

  add_executable(tcptracker src/tcptracker.cpp)
  add_dependencies(tcptracker ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
   target_link_libraries(tcptracker ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

   add_executable(extrinsicCalibration src/extrinsicCalibration.cpp)
   add_dependencies(extrinsicCalibration ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    target_link_libraries(extrinsicCalibration ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

  add_executable(skeleton src/skeleton.cpp src/alglibinternal.cpp
      src/linalg.cpp src/statistics.cpp src/optimization.cpp src/specialfunctions.cpp src/solvers.cpp
      src/interpolation.cpp src/ap.cpp src/alglibmisc.cpp src/dataanalysis.cpp
      src/fasttransforms.cpp src/integration.cpp)

  add_dependencies(skeleton ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
   target_link_libraries(skeleton ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})

   add_dependencies(extrinsicCalibration ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    target_link_libraries(extrinsicCalibration ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})

   add_executable( interpolator src/interpolator.cpp src/alglibinternal.cpp src/linalg.cpp src/statistics.cpp src/optimization.cpp
       src/specialfunctions.cpp src/solvers.cpp src/interpolation.cpp src/ap.cpp
       src/alglibmisc.cpp src/dataanalysis.cpp src/fasttransforms.cpp src/integration.cpp)
   add_dependencies(interpolator ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    target_link_libraries(interpolator ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

    catkin_install_python(PROGRAMS scripts/visualize.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
      )
      catkin_install_python(PROGRAMS scripts/random_pub.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    catkin_install_python(PROGRAMS scripts/parametricCurveFitting.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

  install(TARGETS camera_pub camera_sub skeleton extrinsicCalibration # interpolator
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# statistics specialfunctions
#solvers optimization linalg interpolation integration fasttransforms diffequations dataanalysis ap alglibmisc alglibinternal
