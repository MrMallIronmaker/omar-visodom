cmake_minimum_required(VERSION 3.3)
project(jni)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    com_mrmallironmaker_realtimeoverlay_MainActivity.cpp
    com_mrmallironmaker_realtimeoverlay_MainActivity.h
    Pose.cpp
    Pose.h
    Reference_Frame.h
    test.cpp
    Tracking.cpp
    Tracking.h
    xyz1.cpp)

add_executable(jni ${SOURCE_FILES})