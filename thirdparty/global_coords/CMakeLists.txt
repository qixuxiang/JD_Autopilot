cmake_minimum_required(VERSION 2.8.3)
project(global_coords)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS roscpp angles)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES global_coords
  CATKIN_DEPENDS roscpp angles
  DEPENDS EIGEN
)


include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)


add_executable(converter src/main.cpp src/global_coords.cpp)
target_link_libraries(converter ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
