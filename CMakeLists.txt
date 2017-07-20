cmake_minimum_required(VERSION 2.8.3)
project(roboy_intents_classification)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roboy_communication_cognition
  
)

#  services in the 'srv' folder
# add_service_files(
#   FILES
#   ShowEmotion.srv
# )

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   EmotionMsg.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
  CATKIN_DEPENDS rospy roboy_communication_cognition
)


#############
## Install ##
#############
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 install(PROGRAMS
   scripts/detect_intent.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


## Declare a cpp executable
# add_executable(roboy_face_node src/roboy_face_node.cpp)
# target_link_libraries(
# 	roboy_face_node
# 	${catkin_LIBRARIES}
# )
# add_dependencies(roboy_face_node ${catkin_EXPORTED_TARGETS})

#add_executable(test_client src/test_client.cpp)
#target_link_libraries(
#	test_client
#	${catkin_LIBRARIES}
#)
#add_dependencies(test_client  ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## build talker and listener
#include_directories(
#	${catkin_INCLUDE_DIRS}
#	${GLIB_INCLUDE_DIRS}
#  ${GST_APP_INCLUDE_DIRS}
#)
