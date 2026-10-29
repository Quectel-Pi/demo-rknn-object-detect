cmake_minimum_required(VERSION 3.6)

project(rknn_yolov5_demo)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# skip 3rd-party lib dependencies
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--allow-shlib-undefined")

# install target and libraries
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install/rknn_yolov5_demo_${CMAKE_SYSTEM_NAME})

set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

if(CMAKE_C_COMPILER MATCHES "aarch64")
  set(LIB_ARCH aarch64)
else()
  set(LIB_ARCH armhf)
endif()

include_directories(${CMAKE_SOURCE_DIR})

# rknn api (自包含：3rdparty/librknn_api)
set(RKNN_API_PATH ${CMAKE_SOURCE_DIR}/3rdparty/librknn_api)

if(CMAKE_SYSTEM_NAME STREQUAL "Android")
  set(RKNN_RT_LIB ${RKNN_API_PATH}/${CMAKE_ANDROID_ARCH_ABI}/librknnrt.so)
else()
  set(RKNN_RT_LIB ${RKNN_API_PATH}/${LIB_ARCH}/librknnrt.so)
endif()

include_directories(${RKNN_API_PATH}/include)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty)

# opencv
if(CMAKE_SYSTEM_NAME STREQUAL "Android")
  set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/3rdparty/opencv/OpenCV-android-sdk/sdk/native/jni/abi-${CMAKE_ANDROID_ARCH_ABI})
else()
  if(LIB_ARCH STREQUAL "armhf")
    set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/3rdparty/opencv/opencv-linux-armhf/share/OpenCV)
  else()
    set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/3rdparty/opencv/opencv-linux-aarch64/share/OpenCV)
  endif()
endif()

find_package(OpenCV REQUIRED)

# libjpeg-turbo (for MJPG decode in v4l2 camera mode)
# 用 stb_image.h 单头文件 JPEG 解码器，无外部库依赖。
# 3rdparty/stb/stb_image.h 已包含。
set(STB_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/3rdparty/stb)
include_directories(${STB_INCLUDE_DIR})
message(STATUS "STB include: ${STB_INCLUDE_DIR}")

# rga
# comes from https://github.com/airockchip/librga
set(RGA_PATH ${CMAKE_SOURCE_DIR}/3rdparty/rga/)
if(CMAKE_SYSTEM_NAME STREQUAL "Android")
  set(RGA_LIB ${RGA_PATH}/libs/AndroidNdk/${CMAKE_ANDROID_ARCH_ABI}/librga.so)
else()
  if(CMAKE_C_COMPILER MATCHES "aarch64")
    set(LIB_ARCH aarch64)
  else()
    set(LIB_ARCH armhf)
  endif()

  set(RGA_LIB ${RGA_PATH}/libs/Linux//gcc-${LIB_ARCH}/librga.so)
endif()
include_directories( ${RGA_PATH}/include)

# mpp
set(MPP_PATH ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/mpp)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(MPP_LIBS ${MPP_PATH}/${CMAKE_SYSTEM_NAME}/${LIB_ARCH}/librockchip_mpp.so)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Android")
  set(MPP_LIBS ${MPP_PATH}/${CMAKE_SYSTEM_NAME}/${CMAKE_ANDROID_ARCH_ABI}/libmpp.so)
endif()

include_directories(${MPP_PATH}/include)

# zlmediakit (可选，RTSP 支持，本 demo 不需要，跳过)
# set(ZLMEDIAKIT_PATH ${CMAKE_SOURCE_DIR}/3rdparty/zlmediakit)
set(ZLMEDIAKIT_LIBS "")

if(ZLMEDIAKIT_LIBS)
  add_definitions(-DBUILD_VIDEO_RTSP)
endif()

# gstreamer (real-time display)
# Headers: host gstreamer dev (API-compatible) + buildroot sysroot for aarch64.
# Libs: board trixie gstreamer pulled to local dir (normal libm NEEDED, unlike
# buildroot sysroot which hardcodes /lib64/libm.so.6 missing from toolchain).
get_filename_component(GST_LOCAL_DIR "${CMAKE_SOURCE_DIR}/3rdparty/gst-board" ABSOLUTE)
find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(GSTREAMER gstreamer-1.0 gstreamer-app-1.0)
endif()
if(GSTREAMER_FOUND)
  include_directories(${GSTREAMER_INCLUDE_DIRS})
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L${GST_LOCAL_DIR} -Wl,-rpath-link,${GST_LOCAL_DIR} -Wl,--allow-shlib-undefined")
endif()

set(CMAKE_INSTALL_RPATH "lib")

# rknn_yolov5_demo
include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(rknn_yolov5_demo
  src/main.cc
  src/preprocess.cc
  src/postprocess.cc
)

target_link_libraries(rknn_yolov5_demo
  ${RKNN_RT_LIB}
  ${RGA_LIB}
  ${OpenCV_LIBS}
)

if(MPP_LIBS)
  add_executable(rknn_yolov5_video_demo
    src/main_video.cc
    src/v4l2_capture.cpp
    src/tracker.cpp
    src/event_logger.cpp
    src/postprocess.cc
    utils/mpp_decoder.cpp
    utils/mpp_encoder.cpp
    utils/drawing.cpp
  )
  target_link_libraries(rknn_yolov5_video_demo
    ${RKNN_RT_LIB}
    ${RGA_LIB}
    ${OpenCV_LIBS}
    ${MPP_LIBS}
    ${ZLMEDIAKIT_LIBS}
    ${GSTREAMER_LIBRARIES}
  )
endif()

# install target and libraries
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install/rknn_yolov5_demo_${CMAKE_SYSTEM_NAME})
install(TARGETS rknn_yolov5_demo DESTINATION ./)

install(PROGRAMS ${RKNN_RT_LIB} DESTINATION lib)
install(PROGRAMS ${RGA_LIB} DESTINATION lib)
install(DIRECTORY model/${TARGET_SOC} DESTINATION ./model)
file(GLOB IMAGE_FILES "model/*.jpg")
file(GLOB LABEL_FILE "model/*.txt")
install(FILES ${IMAGE_FILES} DESTINATION ./model/)
install(FILES ${LABEL_FILE} DESTINATION ./model/)

if(MPP_LIBS)
  install(TARGETS rknn_yolov5_video_demo DESTINATION ./)
  install(PROGRAMS ${MPP_LIBS} DESTINATION lib)
endif()

if(ZLMEDIAKIT_LIBS)
  install(PROGRAMS ${ZLMEDIAKIT_LIBS} DESTINATION lib)
endif()
