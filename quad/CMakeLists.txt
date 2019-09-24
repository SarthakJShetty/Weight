cmake_minimum_required(VERSION 2.8.3)
project(quad)

find_package(catkin REQUIRED COMPONENTS
             geometry_msgs
             roscpp mavros
             rospy
             std_msgs
             )

find_package(cmake_modules REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES weight variables
    CATKIN_DEPENDS geometry_msgs mavros roscpp rospy std_msgs
    # DEPENDS system_lib
)

SET(WEIGHT_INCLUDE_DIR include/quad/weight/include/)
SET(WEIGHT_SRC_DIR include/quad/weight/src/)

SET(VARIABLES_DIR include/quad/variables/)

include_directories(include
                    ${VARIABLES_DIR}
                    ${WEIGHT_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    )

add_library(weight ${WEIGHT_INCLUDE_DIR}/weight.hpp)
set_target_properties(weight PROPERTIES LINKER_LANGUAGE CXX)

add_library(variables ${VARIABLES_DIR}/variables.hpp)
set_target_properties(variables PROPERTIES LINKER_LANGUAGE CXX)

add_executable(${PROJECT_NAME}_node src/quad_node.cpp ${WEIGHT_SRC_DIR}/weight.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
