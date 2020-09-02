cmake_minimum_required(VERSION 3.13)

project(offboard_vision)

if(MINGW)
    add_definitions("-D_USE_MATH_DEFINES") # For M_PI
endif()

if(MSVC)
    add_definitions("-std=c++11 -WX -W2")
    add_definitions("-D_USE_MATH_DEFINES") # For M_PI
else()
    add_definitions("-std=c++11 -Wall -Wextra -Werror")
endif()

find_package(MAVSDK REQUIRED)
find_package( OpenCV REQUIRED PATHS ~/opencv-install/opencv-4.4.0)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
	                          
add_executable(offboard_vision
    offboard_vision.cpp
)

target_link_libraries(offboard_vision
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_mocap
    MAVSDK::mavsdk
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

add_custom_target(install2
COMMAND cp offboard_vision ../install)
