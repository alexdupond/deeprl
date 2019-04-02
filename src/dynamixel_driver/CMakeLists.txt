cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_driver)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_library(DXL libdxl_x64_cpp.so)
find_path(DXL_HEADER dynamixel_sdk)
message(STATUS "${DXL}")
message(STATUS "${DXL_HEADER}")

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder 
add_service_files(
  FILES
  SetDynamixelPositions.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynamixel_driver
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories( include ${catkin_INCLUDE_DIRS} )

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/dynamixel_driver.cpp
# )


#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(${PROJECT_NAME}_node src/dynamixel_driver.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  "${DXL}"
)

target_include_directories(${PROJECT_NAME}_node INTERFACE "${DXL_HEADER}")
