cmake_minimum_required(VERSION 2.8.12)
project(CompliantAnkleExperiment)

find_package(YARP REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)
find_package(iDynTree REQUIRED)

include_directories(${yarpWholeBodyInterface_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

include_directories(include)
set(HEADERS include/CompliantAnkleModule.h)
set(SRCS src/main.cpp
         src/CompliantAnkleModule.cpp)

add_executable(${PROJECT_NAME} ${HEADERS} ${SRCS})
target_link_libraries(${PROJECT_NAME} ${yarpWholeBodyInterface_LIBRARIES} ${YARP_LIBRARIES})

add_subdirectory(app)

