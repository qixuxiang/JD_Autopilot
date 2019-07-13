cmake_minimum_required(VERSION 3.0.0)
project(load_pcd_map)
set(CMAKE_CXX_STANDARD 11)

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(CMAKE_EXE_LINKER_FLAGS "-Wl,-rpath,../../../lib,-rpath,.")
endif()

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
)
add_message_files(
    FILES
    JDMetaMap.msg
    JDMap.msg)
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
 add_executable(load_pcd_map_node src/load_pcd_map_node.cpp)
 add_dependencies(load_pcd_map_node ${catkin_EXPORTED_TARGETS})

add_executable(map_engine_node src/map_engine_node.cpp)
add_dependencies(map_engine_node ${catkin_EXPORTED_TARGETS})


if(TARGET MapDataEngine)
  add_library(jdmap::MapDataEngine ALIAS MapDataEngine)
else()
  get_filename_component(jdmap_sdk_dir "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)
  message(STATUS "***jdmap_sdk_dir try ok???=${jdmap_sdk_dir}")
  message("${CMAKE_CURRENT_LIST_DIR}")
  find_package(jdmap-sdk CONFIG REQUIRED  PATHS ${jdmap_sdk_dir}/MapDataEngine/lib/cmake/jdmap-sdk)
endif()

include_directories(${jdmap_sdk_dir}/MapDataEngine/include)
target_link_libraries(load_pcd_map_node
  ${catkin_LIBRARIES}
)
target_link_libraries(map_engine_node
  ${catkin_LIBRARIES}
  jdmap::MapDataEngine
)

#gen vs proprietary file
if(MSVC AND NOT ToolsVersion LESS "11.0")
   set(working_dir "\$(OutDir)")
   set(cmd_args "")
   if(TARGET MapDataEngine)
      get_filename_component(dllpath "${CMAKE_CURRENT_BINARY_DIR}/../../" ABSOLUTE)
     set(dllpath "${dllpath}/Debug;${dllpath}/Release")
   else()
     get_filename_component(dllpath "${CMAKE_SOURCE_DIR}/../../bin" ABSOLUTE)
   endif()
   message(STATUS "***dllpath??=${dllpath}")
endif()

add_definitions(-DVECTORDATA_ENABLED)



