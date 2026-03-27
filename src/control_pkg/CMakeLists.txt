cmake_minimum_required(VERSION 3.8)
project(control_pkg)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(LOWSTATE_DEBUG ON)  # lowstate 数据调试
set(LOWCMD_DEBUG ON)  # lowcmd 数据调试

if(LOWSTATE_DEBUG)
    add_definitions(-DLOWSTATE_DEBUG)
endif()
if(LOWCMD_DEBUG)
    add_definitions(-DLOWCMD_DEBUG)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)


add_executable(start src/start.cpp )

include_directories(${PROJECT_SOURCE_DIR}/include
    /usr/local/include/CSerialPort
)

ament_target_dependencies(start
  rclcpp
  trajectory_msgs
  control_msgs
)

target_link_libraries(
start
UnitreeMotorSDK_Linux64
pthread
cserialport
quadprog
)

install(DIRECTORY launch controller_config ros2_control_config rviz_config
DESTINATION share/${PROJECT_NAME})

install(TARGETS start
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
