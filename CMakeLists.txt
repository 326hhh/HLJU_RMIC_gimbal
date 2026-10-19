cmake_minimum_required(VERSION 3.10)
project(gimbal LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g")
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g3 -Wall -Wextra")

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

# ---------------------------------------------------------------------------
# 内核四层：protocol / transport / device / gimbal
# ---------------------------------------------------------------------------
set(GIMBAL_SOURCES
    src/protocol/zdt_frame.cpp
    src/transport/itransport.cpp
    src/transport/can_transport.cpp
    src/device/zdt_motor.cpp
    src/gimbal/axis.cpp
    src/gimbal/gimbal.cpp
    src/link/udp_control_link.cpp
    src/link/unified_serial.cpp
)

add_library(gimbal_core STATIC ${GIMBAL_SOURCES})
target_include_directories(gimbal_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# termios 串口 / SocketCAN 都在 glibc 里，无需额外链接

# ---------------------------------------------------------------------------
# 可执行文件
# ---------------------------------------------------------------------------
add_executable(gimbal src/main.cpp)
target_link_libraries(gimbal PRIVATE gimbal_core)

# ---------------------------------------------------------------------------
# 自测（纯软件自检：协议向量 + RxDemux 帧分流，不需要真实硬件）
# ---------------------------------------------------------------------------
enable_testing()
add_test(NAME selftest COMMAND gimbal --selftest)

message(STATUS "gimbal: C++${CMAKE_CXX_STANDARD}, build=${CMAKE_BUILD_TYPE}")

# ---------------------------------------------------------------------------
# 车载摄像头 MJPEG 图传（V4L2 直出 MJPG，零转码）
#    ./build/mjpeg_server --size 640x480 --fps 30 --port 8080
#    gimbal_ctl 会自动拉取 http://<host>:8080/stream
# ---------------------------------------------------------------------------
add_executable(mjpeg_server src/video/mjpeg_server.cpp)
target_link_libraries(mjpeg_server PRIVATE pthread)
