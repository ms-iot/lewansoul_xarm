cmake_minimum_required(VERSION 2.8.3)
project(lewansoulxarm)

find_package(catkin REQUIRED
    urdf
    roscpp
    std_msgs
    tf
    sensor_msgs
    message_generation
    interactive_markers
    visualization_msgs
) 
get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_package(
  # INCLUDE_DIRS include
#  LIBRARIES xarm
  DEPENDS Boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(xarm
  src/xarm_ros.cpp
  src/xarm.cpp
  src/hid.cpp
  src/joint.cpp
)
target_include_directories(xarm
  PRIVATE include
)

add_dependencies(xarm
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(xarm
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  SetupAPI.lib
  Hid.lib
  ${CMAKE_CURRENT_SOURCE_DIR}/dep/hidapi-win/x64/hidapi.lib
)

if(MSVC)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/dep/hidapi-win/x64/hidapi.dll
     DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/lewansoulxarm)
endif()

install(TARGETS xarm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
