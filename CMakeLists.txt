cmake_minimum_required(VERSION 2.8.3)
project(lane_extractor)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL 1.4 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  tf2_ros
  tf
  std_msgs
  message_generation
)

add_service_files(FILES
  srvinfo.srv
)
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lane_extractor
  CATKIN_DEPENDS roscpp tf tf2_ros message_runtime 
# DEPENDS system_lib
)





###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
  src/LaneExtractorNode.cpp
  src/CloudProcessing.cpp
  src/LaneExtractor.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(lane_extractor_node
  src/lane_extractor.cpp
)
add_dependencies(lane_extractor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lane_extractor_node
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PROJECT_NAME}
)

add_executable(intensity_cloud_saver
  src/Intensity_cloud_saver.cpp
)
add_dependencies(lane_extractor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(intensity_cloud_saver
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} lane_extractor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
# add_library(LaneExtractor
#   include/lane_extractor/LaneExtractor.h
#   src/LaneExtractor.cpp
# )
# add_dependencies(LaneExtractor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(LaneExtractor
#   ${catkin_LIBRARIES}
# )

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )


