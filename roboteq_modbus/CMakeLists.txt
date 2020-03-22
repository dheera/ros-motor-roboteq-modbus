cmake_minimum_required(VERSION 2.8.3)
project(roboteq_modbus)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg serial)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(roboteq_node src/roboteq_node.cpp src/roboteq_device.cpp)
target_link_libraries(roboteq_node ${catkin_LIBRARIES})
add_dependencies(roboteq_node ${catkin_EXPORTED_TARGETS})

install(TARGETS
   roboteq_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

