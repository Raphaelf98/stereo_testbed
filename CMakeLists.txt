cmake_minimum_required(VERSION 3.0.2)
project(package1)

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


   add_executable(extrinsicCalibration src/extrinsicCalibration.cpp)
   add_dependencies(extrinsicCalibration ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    target_link_libraries(extrinsicCalibration ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

    add_executable(skeleton src/skeleton.cpp )
  add_dependencies(skeleton ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
   target_link_libraries(skeleton ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})

   add_dependencies(extrinsicCalibration ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    target_link_libraries(extrinsicCalibration ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})


    catkin_install_python(PROGRAMS scripts/parametricCurveFitting.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

  install(TARGETS camera_pub camera_sub skeleton extrinsicCalibration
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

