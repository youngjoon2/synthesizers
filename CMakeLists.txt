cmake_minimum_required(VERSION 2.8.3)
project(synthesizers)

add_compile_options(-std=c++17)

find_package(PCL 1.8 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  pcl_ros 
)

# cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
# project(MY_GRAND_PROJECT)
# find_package(PCL 1.8 REQUIRED COMPONENTS common io)
# include_directories(include ${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})
# add_executable(main src/main.cpp)
# target_link_libraries(main ${PCL_LIBRARIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(main src/main.cpp)
add_dependencies(main ${catkin_EXPORTED_TARGETS})
target_link_libraries(main ${catkin_LIBRARIES})
 
install(TARGETS main 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# include header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
