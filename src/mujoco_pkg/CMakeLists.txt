cmake_minimum_required(VERSION 3.8)
project(mujoco_pkg)
set(OpenGL_GL_PREFERENCE "GLVND")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(MUJOCO_FOLDER /opt/mujoco/lib/cmake)


find_package(mujoco REQUIRED PATHS ${MUJOCO_FOLDER} NO_DEFAULT_PATH)
if (mujoco_FOUND)
    message(STATUS "Find mujoco:  ${mujoco_INCLUDE_DIRS}, ${mujoco_LIBRARIES}")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include
)

set(CMAKE_INSTALL_RPATH "/opt/mujoco/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

add_executable(mj src/mj.cc src/ioros.cpp)
add_executable(mujoco src/mujoco.cpp)
# target_link_libraries(mj mujoco::mujoco glut GL GLU glfw pthread)
target_link_libraries(mj mujoco::mujoco glut GL GLU glfw pthread)
target_link_libraries(mujoco mujoco::mujoco glut GL GLU glfw)

ament_target_dependencies(mj 
  rclcpp
  std_msgs
  sensor_msgs
)
ament_target_dependencies(mujoco 
  rclcpp
  std_msgs
  sensor_msgs
)
install(TARGETS mj mujoco
DESTINATION lib/${PROJECT_NAME}
)
ament_package()
 