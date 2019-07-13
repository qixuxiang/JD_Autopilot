cmake_minimum_required(VERSION 3.0.0)
PROJECT(TileConsoleTest)
set(CMAKE_CXX_STANDARD 11)

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(CMAKE_EXE_LINKER_FLAGS "-Wl,-rpath,../../../lib,-rpath,.")
endif()

add_executable(TileConsoleTest  
               TileConsoleTest.cpp
               )
if(TARGET MapDataEngine)
  add_library(jdmap::MapDataEngine ALIAS MapDataEngine)
else()
  get_filename_component(jdmap_sdk_dir "${CMAKE_CURRENT_LIST_DIR}/../../" ABSOLUTE)
  message(STATUS "***jdmap_sdk_dir try ok???=${jdmap_sdk_dir}")
  find_package(jdmap-sdk CONFIG REQUIRED  PATHS ${jdmap_sdk_dir}/MapDataEngine/lib/cmake/jdmap-sdk)
endif()

include_directories(${jdmap_sdk_dir}/MapDataEngine/include)
target_link_libraries(TileConsoleTest jdmap::MapDataEngine)
set_target_properties(TileConsoleTest PROPERTIES FOLDER "Examples")

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

if(TARGET MapDataEngine)
  message(STATUS "***TARGET MapDataEngine***")

  #3.install binary
  install(DIRECTORY ${PROJECT_BINARY_DIR}/
          DESTINATION Examples/TileConsoleTest/bin
          FILE_PERMISSIONS  OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ
          COMPONENT examples
          FILES_MATCHING
          PATTERN "TileConsoleTest"
          #PATTERN "JMap.xml"
          PATTERN "CMakeFiles" EXCLUDE
          )
  #3.install source files
  install(DIRECTORY ./
          DESTINATION  Examples/TileConsoleTest
          COMPONENT examples
          )  
endif()
