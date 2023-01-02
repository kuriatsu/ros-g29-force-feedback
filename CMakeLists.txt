cmake_minimum_required(VERSION 2.8.3)
project(ros_g29_force_feedback)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
)

add_message_files(
    FILES
    ForceFeedback.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES g29_ff
    CATKIN_DEPENDS roscpp rospy message_runtime
    #  DEPENDS system_lib
)

include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/g29_force_feedback.cpp)
add_dependencies(${PROJECT_NAME}_node g29_force_feedback_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
