cmake_minimum_required(VERSION 3.8)
project(ros2_orb_slam3)

# Make sure to set this path before building the  
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/") # Must be set to match your installation

# Must use C++17 to make it compatible with rclcpp
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Check C++17 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)

if (COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(STATUS "Using flag -std=c++17.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif ()

#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules) # REDUNDANT?

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(your_custom_msg_interface REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(Pangolin REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_srvs REQUIRED)

#find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_common REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(image_geometry REQUIRED)

set(dependencies
        rclcpp
        rclpy
        std_msgs
        sensor_msgs
        cv_bridge
        image_common
        image_transport
        tf2_geometry_msgs
        tf2_ros
        image_geometry
        std_srvs
)


find_package(OpenCV 4.2 QUIET)
message("-- Found OpenCV: ${OpenCV_VERSION} (${OpenCV_DIR})")
if (NOT OpenCV_FOUND)
    find_package(OpenCV 4.0 QUIET)
    if (NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 4.0 not found.")
    endif ()
endif ()

find_package(Eigen3 3.1.0 REQUIRED)

# Header file locations [C++ node]
include_directories(${OpenCV_INCLUDE_DIRS}) # include headers for pangolin


# ORB-SLAM3 includes [VSLAM library]
include_directories(
        include
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/orb_slam3
        ${PROJECT_SOURCE_DIR}/orb_slam3/include
        ${PROJECT_SOURCE_DIR}/orb_slam3/include/CameraModels
        ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty
        ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/Sophus
        ${Eigen3_INCLUDE_DIR} # Include headers for eigen3
        ${Pangolin_INCLUDE_DIRS} # include headers for pangolin
        ${ament_INCLUDE_DIRS}
)


# EXTRAS FROM ORBSLAM3
include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/CMakeLists.txt)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/orb_slam3/lib)

#rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

#* ORB SLAM3 as a shared library
# Whenever you add a new .h, .hpp or .cc, .cpp file in the ros2_orb_slam3/orb_slam3/include and os2_orb_slam3/orb_slam3/src directories, make sure to add them here as shown below
add_library(orb_slam3_ros_core SHARED
        orb_slam3/src/System.cc
        orb_slam3/src/Tracking.cc
        orb_slam3/src/LocalMapping.cc
        orb_slam3/src/LoopClosing.cc
        orb_slam3/src/ORBextractor.cc
        orb_slam3/src/ORBmatcher.cc
        orb_slam3/src/FrameDrawer.cc
        orb_slam3/src/Converter.cc
        orb_slam3/src/MapPoint.cc
        orb_slam3/src/KeyFrame.cc
        orb_slam3/src/Atlas.cc
        orb_slam3/src/Map.cc
        orb_slam3/src/MapDrawer.cc
        orb_slam3/src/Optimizer.cc
        orb_slam3/src/Frame.cc
        orb_slam3/src/KeyFrameDatabase.cc
        orb_slam3/src/Sim3Solver.cc
        orb_slam3/src/Viewer.cc
        orb_slam3/src/ImuTypes.cc
        orb_slam3/src/G2oTypes.cc
        orb_slam3/src/CameraModels/Pinhole.cpp
        orb_slam3/src/CameraModels/KannalaBrandt8.cpp
        orb_slam3/src/OptimizableTypes.cpp
        orb_slam3/src/MLPnPsolver.cpp
        orb_slam3/src/GeometricTools.cc
        orb_slam3/src/TwoViewReconstruction.cc
        orb_slam3/src/Config.cc
        orb_slam3/src/Settings.cc
        orb_slam3/include/System.h
        orb_slam3/include/Tracking.h
        orb_slam3/include/LocalMapping.h
        orb_slam3/include/LoopClosing.h
        orb_slam3/include/ORBextractor.h
        orb_slam3/include/ORBmatcher.h
        orb_slam3/include/FrameDrawer.h
        orb_slam3/include/Converter.h
        orb_slam3/include/MapPoint.h
        orb_slam3/include/KeyFrame.h
        orb_slam3/include/Atlas.h
        orb_slam3/include/Map.h
        orb_slam3/include/MapDrawer.h
        orb_slam3/include/Optimizer.h
        orb_slam3/include/Frame.h
        orb_slam3/include/KeyFrameDatabase.h
        orb_slam3/include/Sim3Solver.h
        orb_slam3/include/Viewer.h
        orb_slam3/include/ImuTypes.h
        orb_slam3/include/G2oTypes.h
        orb_slam3/include/CameraModels/GeometricCamera.h
        orb_slam3/include/CameraModels/Pinhole.h
        orb_slam3/include/CameraModels/KannalaBrandt8.h
        orb_slam3/include/OptimizableTypes.h
        orb_slam3/include/MLPnPsolver.h
        orb_slam3/include/GeometricTools.h
        orb_slam3/include/TwoViewReconstruction.h
        orb_slam3/include/SerializationUtils.h
        orb_slam3/include/Config.h
        orb_slam3/include/Settings.h)

#set_target_properties(orb_slam3_ros_core PROPERTIES VERSION "${orb_slam3_ros_core_VERSION}") # TODO need to findout why this is required

ament_target_dependencies(orb_slam3_ros_core
        rclcpp
        std_msgs
        sensor_msgs
        # ${dependencies}
)

#add_subdirectory(orb_slam3/Thirdparty/g2o)

# Link libraries
target_link_libraries(orb_slam3_ros_core
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
#        ${PROJECT_SOURCE_DIR}/orbs_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
#        ${PROJECT_SOURCE_DIR}/orbs_slam3/Thirdparty/g2o/lib/libg2o.so
                DBoW2
                g2o
#                -lboost_system
#        -lboost_serialization
#        -lcrypto
)

message(STATUS "Compile With map save/load function")
find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
    message(FATAL_ERROR "Can't find libboost_serialization")
endif ()
target_link_libraries(orb_slam3_ros_core ${BOOST_SERIALIZATION})

#target_link_libraries(orb_slam3_ros_core "${cpp_typesupport_target}")

# Find the .so files provided in ros2_orb_slam3/orb_slam3/Thirdparty projects
#set(DBoW2_PATH "${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so")
#set(g2o_PATH "${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so")


# Install .so files to the lib directory in the install space
#install(FILES ${DBoW2_PATH} DESTINATION lib)
#install(FILES ${g2o_PATH} DESTINATION lib)

add_executable(stereo src/stereo/stereo.cpp src/stereo/stereo-slam-node.cpp)
ament_target_dependencies(stereo ${dependencies})
target_link_libraries(stereo orb_slam3_ros_core ${OpenCV_LIBS}) # Link a node with the internal shared library

add_executable(mono src/monocular/mono.cpp src/monocular/monocular-slam-node.cpp)
ament_target_dependencies(mono ${dependencies})
target_link_libraries(mono orb_slam3_ros_core ${OpenCV_LIBS}) # Link a node with the internal shared library

# Install all the header files in package/package/include
install(DIRECTORY include/
        DESTINATION include
)

# Install our node and library
install(TARGETS orb_slam3_ros_core
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION include/${PROJECT_NAME}
)

# Tell downstream packages where to find the headers
ament_export_include_directories(include)

# Help downstream packages to find transitive dependencies
ament_export_dependencies(
        orb_slam3_ros_core
        ${dependencies}
)


# Python node
# Install Python modules, submodules
ament_python_install_package(${PROJECT_NAME}) # Install the modules in ros2_orb_slam3/ros2_orb_slam3 folder


# Install Python executibles
install(PROGRAMS
        scripts/mono_driver_node.py
        DESTINATION lib/${PROJECT_NAME}
)


#install(TARGETS stereo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY orb_slam3/Vocabulary/
        DESTINATION share/${PROJECT_NAME}/orb_slam2/Vocabulary
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS stereo mono RUNTIME DESTINATION lib/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_dependencies(
        ${dependencies}
        rosidl_default_runtime
        #  DBoW2
        #  g2o
)
#ament_export_libraries(orb_slam2_ros_core libDBoW2 libg2o)
ament_export_libraries(orb_slam3_ros_core)

ament_package()


