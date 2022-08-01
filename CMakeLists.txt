cmake_minimum_required(VERSION 2.8)
project(hikrobot_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
    camera_info_manager 
    cv_bridge 
    roslib
    pcl_conversions
    pcl_ros
    image_transport 
    message_generation 
    sensor_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
)

generate_messages(DEPENDENCIES sensor_msgs)

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)

include_directories( 
    ${OpenCV_INCLUDES_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${GLFW_INCLUDE_DIRS}
    ${Boost_INCLUDES}
  )

link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})


catkin_package(CATKIN_DEPENDS cv_bridge image_transport camera_info_manager message_runtime sensor_msgs INCLUDE_DIRS include)


include_directories(
  include
  node/common/include
  node/common/include/glad
  node/common/include/GLFW
  node/common/include/KHR
  ${catkin_INCLUDE_DIRS}
)

link_directories(lib)

add_library(hik_cam node/Mv3dRGBD_node.cpp)
target_link_libraries(hik_cam
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OPENGL_LIBRARIES} 
    ${GLFW_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/lib/libFormatConversion.so
    ${PROJECT_SOURCE_DIR}/lib/libMv3dRgbd.so
    ${PROJECT_SOURCE_DIR}/lib/libGCBase_gcc421_v3_0.so
    ${PROJECT_SOURCE_DIR}/lib/libGenApi_gcc421_v3_0.so
    ${PROJECT_SOURCE_DIR}/lib/libLog_gcc421_v3_0.so
    ${PROJECT_SOURCE_DIR}/lib/liblog4cpp_gcc421_v3_0.so
    ${PROJECT_SOURCE_DIR}/lib/libMathParser_gcc421_v3_0.so
    ${PROJECT_SOURCE_DIR}/lib/libMediaProcess.so
    ${PROJECT_SOURCE_DIR}/lib/libMvCameraControl.so
    ${PROJECT_SOURCE_DIR}/lib/libMvCameraControlWrapper.so.1.0.1.0
    ${PROJECT_SOURCE_DIR}/lib/libMVGigEVisionSDK.so
    ${PROJECT_SOURCE_DIR}/lib/libMVRender.so
    ${PROJECT_SOURCE_DIR}/lib/libMvUsb3vTL.so
    ${PROJECT_SOURCE_DIR}/lib/libNodeMapData_gcc421_v3_0.so
    ${PROJECT_SOURCE_DIR}/lib/libXmlParser_gcc421_v3_0.so
    ${PROJECT_SOURCE_DIR}/node/common/lib/linux64
)

add_subdirectory(third-party/apriltags)

add_library(apriltag_lib SHARED
    third-party/apriltags/CameraUtil.cpp
    third-party/apriltags/DebugImage.cpp
    third-party/apriltags/Geometry.cpp 
    third-party/apriltags/GrayModel.cpp
    third-party/apriltags/MathUtil.cpp 
    third-party/apriltags/Refine.cpp
    third-party/apriltags/TagDetector.cpp 
    third-party/apriltags/TagFamily.cpp
    third-party/apriltags/TagFamilies.cpp
    third-party/apriltags/UnionFindSimple.cpp
)

add_executable(Mv3dRgbd_node node/Mv3dRGBD_node.cpp)
target_link_libraries(Mv3dRgbd_node hik_cam)

add_executable(subscriber node/subscriber.cpp)
target_link_libraries(subscriber 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})

add_executable(camera_ui node/camera_ui.cpp)
target_link_libraries(camera_ui 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})

add_executable(camera_pose node/camera_pose.cpp)
target_link_libraries(camera_pose 
    apriltag_lib 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})

add_executable(aligned_depth node/aligned_depth.cpp)
target_link_libraries(aligned_depth
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES} 
    ${PCL_LIBRARIES}
)
