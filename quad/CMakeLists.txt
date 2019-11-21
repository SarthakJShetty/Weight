cmake_minimum_required(VERSION 2.8.3)
project(quad)

find_package(catkin REQUIRED COMPONENTS
             geometry_msgs
             roscpp mavros
             rospy
             std_msgs
             mavros_msgs
             )

find_package(cmake_modules REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES weight variables lawnmower survivor
    CATKIN_DEPENDS geometry_msgs mavros roscpp rospy std_msgs mavros_msgs
    # DEPENDS system_lib
)

SET(WEIGHT_INCLUDE_DIR include/quad/weight/include/)
SET(WEIGHT_SRC_DIR include/quad/weight/src/)
SET(LAWNMOWER_INCLUDE_DIR include/quad/lawnmower/include/)
SET(LAWNMOWER_SRC_DIR include/quad/lawnmower/src/)
SET(SURVIVOR_INCLUDE_DIR include/quad/survivor/include/)
SET(SURVIVOR_SRC_DIR include/quad/survivor/src/)

SET(VARIABLES_DIR include/quad/variables/)

include_directories(include
                    ${VARIABLES_DIR}
                    ${WEIGHT_INCLUDE_DIR}
                    ${LAWNMOWER_INCLUDE_DIR}
                    ${SURVIVOR_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    )

add_library(weight ${WEIGHT_INCLUDE_DIR}/weight.hpp)
set_target_properties(weight PROPERTIES LINKER_LANGUAGE CXX)

add_library(lawnmower ${LAWNMOWER_INCLUDE_DIR}/lawnmower.hpp)
set_target_properties(lawnmower PROPERTIES LINKER_LANGUAGE CXX)

add_library(survivor ${SURVIVOR_INCLUDE_DIR}/survivor.hpp)
set_target_properties(survivor PROPERTIES LINKER_LANGUAGE CXX)

add_library(variables ${VARIABLES_DIR}/variables.hpp)
set_target_properties(variables PROPERTIES LINKER_LANGUAGE CXX)

add_executable(${PROJECT_NAME}_node src/quad_node.cpp ${WEIGHT_SRC_DIR}/weight.cpp ${LAWNMOWER_SRC_DIR}/lawnmower.cpp ${SURVIVOR_SRC_DIR}/survivor.cpp)
add_executable(${PROJECT_NAME}_observer_node src/observer_node.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_observer_node ${catkin_LIBRARIES})