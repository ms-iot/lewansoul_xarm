cmake_minimum_required(VERSION 3.5)
project(lewansoulxarm)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)


find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  include
)

add_executable(xarm
  src/xarm_ros.cpp
  src/xarm.cpp
  src/hid.cpp
  src/joint.cpp
)

ament_target_dependencies(xarm
  ament_index_cpp
  rclcpp
  std_msgs
  sensor_msgs
  tf2_ros
  geometry_msgs
)

target_link_libraries(xarm
  ${Boost_LIBRARIES}
  Hid.lib
  ${CMAKE_CURRENT_SOURCE_DIR}/dep/hidapi-win/x64/hidapi.lib
)

if(MSVC)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/dep/hidapi-win/x64/hidapi.dll
     DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/lewansoulxarm)
endif()

install(TARGETS xarm
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

ament_package()