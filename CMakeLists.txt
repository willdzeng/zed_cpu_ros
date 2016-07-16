cmake_minimum_required(VERSION 2.8.3)
project(zed_cpu_ros)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            tf
            cv_bridge
            image_transport
            camera_info_manager
        )
# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
    # INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS
)

# add_library(zed_cpu_ros src/zed_cpu_ros.cpp)
# target_link_libraries(zed_cpu_ros ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(zed_cpu_ros arti_msgs_gencpp)

add_executable(zed_cpu_ros src/zed_cpu_ros.cpp)
target_link_libraries(zed_cpu_ros ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

# install(TARGETS 
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   PATTERN ".svn" EXCLUDE
# )