
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0)
project(BlurEstimate C CXX)

set ( CMAKE_BUILD_TYPE "Release")

set( CMAKE_CXX_FLAGS "-std=c++11" )
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")

# set( CMAKE_CXX_FLAGS "-O3" )


add_definitions(-DMULTI_API_VERSION)
find_package( OpenCV REQUIRED )


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


include_directories(${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/driver/include ${PROJECT_SOURCE_DIR}/driver/src/hid_control)
add_executable(blur_estimate blur_estimate.cpp)

set(TARGET_LIBRARY_PATH 
    ${PROJECT_SOURCE_DIR}/libs/libHELLO.so
    ${PROJECT_SOURCE_DIR}/libs/libUVC_SDK.so
    ${PROJECT_SOURCE_DIR}/libs/libBMP.so
    ${PROJECT_SOURCE_DIR}/libs/libHIDCONTROL.so)

target_link_libraries(blur_estimate ${TARGET_LIBRARY_PATH} ${OpenCV_LIBS} -lusb-1.0 -ludev -lpthread  -lGL -lGLU -lglut  -fpermissive -w -O3)