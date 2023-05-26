
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0)
project(BlurEstimate C CXX)

set ( CMAKE_BUILD_TYPE "Release")

set( CMAKE_CXX_FLAGS "-std=c++11" )
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")

# set( CMAKE_CXX_FLAGS "-O3" )


add_definitions(-DMULTI_API_VERSION)
find_package( OpenCV REQUIRED )
find_package( PCL 1.3 REQUIRED COMPONENTS common io )


set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/libs)
add_library(BMP SHARED 
    ${PROJECT_SOURCE_DIR}/driver/include/bmp.h
    ${PROJECT_SOURCE_DIR}/driver/src/bmp.c)
target_link_libraries(BMP -lusb-1.0)

add_library(UVC_SDK SHARED 
    ${PROJECT_SOURCE_DIR}/driver/include/uvc_cam_sdk.h
    ${PROJECT_SOURCE_DIR}/driver/src/uvc_cam_sdk.c)
target_link_libraries(UVC_SDK -lusb-1.0)

add_library(HELLO SHARED 
    ${PROJECT_SOURCE_DIR}/driver/include/hello.h
    ${PROJECT_SOURCE_DIR}/driver/src/hello.c)

add_library(HIDCONTROL SHARED 
    ${PROJECT_SOURCE_DIR}/driver/src/hid_control/hid_control.cpp
    ${PROJECT_SOURCE_DIR}/driver/src/hid_control/hid.c
    ${PROJECT_SOURCE_DIR}/driver/src/uvc_cam_sdk.c
    ${PROJECT_SOURCE_DIR}/driver/include/uvc_cam_sdk.h
    ${PROJECT_SOURCE_DIR}/driver/src/hid_control/hid_control.hpp
    ${PROJECT_SOURCE_DIR}/driver/src/hid_control/hidapi.h
    ${PROJECT_SOURCE_DIR}/driver/src/hid_control/libusb.h
    ${PROJECT_SOURCE_DIR}/driver/include/ccommon.h
    ${PROJECT_SOURCE_DIR}/driver/include/cfunctions.h
    ${PROJECT_SOURCE_DIR}/driver/include/common.h
    ${PROJECT_SOURCE_DIR}/driver/include/config.h
    ${PROJECT_SOURCE_DIR}/driver/include/const_xslam.h
    ${PROJECT_SOURCE_DIR}/driver/include/container.h
    ${PROJECT_SOURCE_DIR}/driver/include/eigen.hpp
    ${PROJECT_SOURCE_DIR}/driver/include/functions.h
    ${PROJECT_SOURCE_DIR}/driver/include/geometry.h
    ${PROJECT_SOURCE_DIR}/driver/include/hidCommonMode.h
    ${PROJECT_SOURCE_DIR}/driver/include/misc.h)
target_link_libraries(HIDCONTROL -lusb-1.0 -ludev -lpthread  -lGL -lGLU -lglut  -fpermissive -w -O3)

# example

include_directories(${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/driver/include ${PROJECT_SOURCE_DIR}/driver/src/hid_control)
add_executable(blur_estimate ${PROJECT_SOURCE_DIR}/demo/blur_estimate.cpp)
add_executable(demo ${PROJECT_SOURCE_DIR}/demo/demo.cpp)
add_executable(pointShow ${PROJECT_SOURCE_DIR}/demo/pointShow.cpp)
add_executable(fastDetector ${PROJECT_SOURCE_DIR}/demo/fastDetector.cpp)
add_executable(yuv420torgb ${PROJECT_SOURCE_DIR}/demo/yuv420torgb.cpp )

include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(depth2pointcloud ${PROJECT_SOURCE_DIR}/demo/depth2pointcloud.cpp)
add_executable(showPointCloud ${PROJECT_SOURCE_DIR}/demo/showPointCloud.cpp)

set(TARGET_LIBRARY_PATH 
    ${PROJECT_SOURCE_DIR}/libs/libHELLO.so
    ${PROJECT_SOURCE_DIR}/libs/libUVC_SDK.so
    ${PROJECT_SOURCE_DIR}/libs/libBMP.so
    ${PROJECT_SOURCE_DIR}/libs/libHIDCONTROL.so)


# show pointcloud ,can not find libpcl_visualization.so
set (PCL_VISUALIZATION_LIB
    "/usr/local/lib/libpcl_visualization.so")

target_link_libraries(blur_estimate ${TARGET_LIBRARY_PATH} ${OpenCV_LIBS} -lusb-1.0 -ludev -lpthread  -lGL -lGLU -lglut  -fpermissive -w -O3)
target_link_libraries(demo ${TARGET_LIBRARY_PATH} ${OpenCV_LIBS} -lusb-1.0 -ludev -lpthread  -lGL -lGLU -lglut  -fpermissive -w -O3)
target_link_libraries(pointShow ${OpenCV_LIBS} -fpermissive -w -O3)
target_link_libraries(fastDetector ${OpenCV_LIBS} ${TARGET_LIBRARY_PATH} -lusb-1.0 -ludev -lpthread  -lGL -lGLU -lglut -fpermissive -w -O3)
target_link_libraries(yuv420torgb ${TARGET_LIBRARY_PATH} ${OpenCV_LIBS} -lusb-1.0 -ludev -lpthread  -lGL -lGLU -lglut  -fpermissive -w -O3)
# ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} 
target_link_libraries(depth2pointcloud ${OpenCV_LIBS}  ${PCL_LIBRARIES} -fpermissive -w -O3)
target_link_libraries(showPointCloud ${PCL_LIBRARIES} ${PCL_VISUALIZATION_LIB} -fpermissive -w -O3)

