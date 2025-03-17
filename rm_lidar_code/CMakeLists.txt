cmake_minimum_required(VERSION 3.8)
project(mid70pp)

set(SDK_LIBRARY livoxsdk_static)
add_library(${SDK_LIBRARY} STATIC "")

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(pub src/mid70pp_node.cpp)
ament_target_dependencies(pub rclcpp std_msgs sensor_msgs)
# target_link_libraries(pub
#          PRIVATE
#          livoxsdk_static
#          )
target_link_libraries(pub 
livoxsdk_static
         )

set(LIVOX_SDK_MAJOR_VERSION "2")
set(LIVOX_SDK_MINOR_VERSION "3")
set(LIVOX_SDK_PATCH_VERSION "0")
set(LIVOX_SDK_VERSION_STRING "${LIVOX_SDK_MAJOR_VERSION}.${LIVOX_SDK_MINOR_VERSION}.${LIVOX_SDK_PATCH_VERSION}")

target_include_directories(${SDK_LIBRARY}
        PUBLIC
        include
        include/third_party/FastCRC
        include/third_party/spdlog
        include/third_party/cmdline
        PRIVATE
        src)

set_target_properties(${SDK_LIBRARY} PROPERTIES PUBLIC_HEADER "include/livox_def.h;include/livox_sdk.h")

if(WIN32)
  set(PLATFORM win)
else(WIN32)
  set(PLATFORM unix)
endif (WIN32)


target_compile_options(${SDK_LIBRARY}
        PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall -Werror -Wno-c++11-long-long>
        PRIVATE $<$<CXX_COMPILER_ID:AppleClang>:-Wno-unknown-pragmas -Wall -Werror -Wno-c++11-long-long>
        PRIVATE $<$<CXX_COMPILER_ID:Clang>:-Wno-unknown-pragmas -Wall -Werror -Wno-c++11-long-long>)

target_sources(${SDK_LIBRARY}
        PRIVATE
        src/third_party/FastCRC/FastCRC_tables.hpp
        src/third_party/FastCRC/FastCRCsw.cpp
        src/base/io_loop.h
        src/base/io_loop.cpp
        src/base/thread_base.h
        src/base/thread_base.cpp
        src/base/io_thread.h
        src/base/io_thread.cpp
        src/base/network/network_util.h
        src/base/network/${PLATFORM}/network_util.cpp
        src/base/logging.h
        src/base/logging.cpp
        src/base/noncopyable.h
        src/livox_sdk.cpp
        src/device_discovery.h
        src/device_discovery.cpp
        src/device_manager.h
        src/device_manager.cpp
        src/comm/comm_port.cpp
        src/comm/sdk_protocol.h
        src/comm/sdk_protocol.cpp
        src/data_handler/data_handler.h
        src/data_handler/data_handler.cpp
        src/data_handler/hub_data_handler.h
        src/data_handler/hub_data_handler.cpp
        src/data_handler/lidar_data_handler.h
        src/data_handler/lidar_data_handler.cpp
        src/command_handler/command_handler.h
        src/command_handler/command_handler.cpp
        src/command_handler/command_channel.h
        src/command_handler/command_channel.cpp
        src/command_handler/hub_command_handler.h
        src/command_handler/hub_command_handler.cpp
        src/command_handler/lidar_command_handler.h
        src/command_handler/lidar_command_handler.cpp
        src/command_handler/command_impl.cpp
        src/command_handler/command_impl.h
        src/base/multiple_io/multiple_io_base.h
        src/base/multiple_io/multiple_io_base.cpp
        src/base/multiple_io/multiple_io_factory.h
        src/base/multiple_io/multiple_io_epoll.h
        src/base/multiple_io/multiple_io_epoll.cpp
        src/base/multiple_io/multiple_io_poll.h
        src/base/multiple_io/multiple_io_poll.cpp
        src/base/multiple_io/multiple_io_select.h
        src/base/multiple_io/multiple_io_select.cpp
        src/base/multiple_io/multiple_io_kqueue.h
        src/base/multiple_io/multiple_io_kqueue.cpp
        src/base/wake_up/wake_up_pipe.h
        src/base/wake_up/${PLATFORM}/wake_up_pipe.cpp)



if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
# 取消注释以下部分以手动填写其他依赖项。
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 以下行跳过了检查版权声明的linter
  # 当所有源文件都添加了版权声明和许可证时，请注释掉该行
  set(ament_cmake_copyright_FOUND TRUE)
  # 以下行跳过了cpplint（仅在git仓库中有效）
  # 当此包位于git仓库中，并且所有源文件都添加了版权声明和许可证时，请注释掉该行
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#ament_target_dependencies(<executable-name> [dependencies])


install(TARGETS ${SDK_LIBRARY}
        PUBLIC_HEADER DESTINATION include
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

install(TARGETS
pub
DESTINATION lib/${PROJECT_NAME})





ament_package()
