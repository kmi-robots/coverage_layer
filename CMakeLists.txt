cmake_minimum_required(VERSION 2.8.3)
project(coverage_layer)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  tf
  costmap_2d
  std_srvs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/coverage_layer.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS coverage_layer
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES costmap_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(coverage_node src/coverage_node.cpp src/coverage.cpp)
target_link_libraries(coverage_node ${catkin_LIBRARIES} ${PROJECT_NAME})
