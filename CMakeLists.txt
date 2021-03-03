cmake_minimum_required(VERSION 3.5)
project(rec3d)


set( CMAKE_CXX_STANDARD 14 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_FLAGS "-Wall -m64 -O0" )

set( IS_SIMULATION true )

add_definitions( -DIS_SIMULATION=${IS_SIMULATION} )

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    cv_bridge
    image_transport
)


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
    CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport   
)

set( OpenCV_DIR "/usr/local/lib/cmake/opencv4" )
set( EIGEN3_INCLUDE_DIR "/usr/include/eigen3" )
###########
## Build ##
###########

find_package(OpenCV 4 REQUIRED)

if (NOT ${OpenCV_FOUND})
    message(FATAL_ERROR "Couldn't found OpenCV")
endif()

message(STATUS "OpenCV ${OpenCV_VERSION} FOUND")

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

# BEHAVIOR TREE
find_package(BehaviorTreeV3 REQUIRED)



link_libraries("/usr/local/lib")

add_library( ${PROJECT_NAME}-uav SHARED src/Uav.cpp)
target_link_libraries( ${PROJECT_NAME}-uav ${OpenCV_LIBS} ${catkin_LIBRARIES} )
set_target_properties( ${PROJECT_NAME}-uav PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib )

set ( PROJECT_LIBS ${PROJECT_NAME}-uav )

# add_executable(
#     behavior_tree_node 
#     src/BehaviorFactory.cpp
#     src/UavInterface.cpp
# )
# target_link_libraries(behavior_tree_node BT::behaviortree_cpp_v3 ${PROJECT_LIBS} )

add_executable(
    main_tree_node 
    src/main.cpp
    src/UavInterface.cpp
)
target_link_libraries(main_tree_node BT::behaviortree_cpp_v3 ${PROJECT_LIBS} )


# add_executable(mission_planner_cpp src/mission_planner.cpp)
# target_link_libraries( mission_planner_cpp ${OpenCV_LIBS} ${catkin_LIBRARIES} ${PROJECT_LIBS} )


# set(SOURCES src/main.cpp)
# add_executable(${PROJECT_NAME} ${SOURCES})
# add_executable(reconstruction_node src/reconstruct.cpp src/matcher.cpp)
# add_executable(simple_subscriber src/simple_subscriber.cpp)


# target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES} )
# target_link_libraries( reconstruction_node ${OpenCV_LIBS} ${catkin_LIBRARIES} )
# target_link_libraries( simple_subscriber ${OpenCV_LIBS} ${catkin_LIBRARIES} )

