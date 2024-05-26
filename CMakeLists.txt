cmake_minimum_required(VERSION 3.20)
project(BatchInformedVines VERSION 0.1)

# 基础准备
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ompl 1.6 REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS math_c99)

if(${OMPL_FOUND})
    message(STATUS "Found OMPL v${OMPL_VERSION}")
endif()

if(${Eigen3_FOUND})
    message(STATUS "Found Eigen v${Eigen3_VERSION}")
endif()

if(${Boost_FOUND})
    message(STATUS "Found Boost v${Boost_VERSION}")
endif()

# 第一个 target: demo
add_executable(ompl-demo 
                  ompl-demo/ompl-demo.cpp
                  bivstar/BIVstar.cpp
                  bivstar/src/ImplicitGraph.cpp
                  bivstar/src/SearchQueue.cpp
                  bivstar/src/Vertex.cpp
)

target_include_directories(ompl-demo
                            PRIVATE
                            ${OMPL_INCLUDE_DIRS}
                            ${Eigen_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIR}
                            ${CMAKE_HOME_DIRECTORY}/bivstar
)


target_link_libraries(ompl-demo
                        PRIVATE
                        ${OMPL_LIBRARIES}
                        ${Boost_LIBRARIES}
)

add_executable(basic-test
                bivstar/tests/testPCA.cpp
)

target_include_directories(basic-test
                            PRIVATE
                            ${OMPL_INCLUDE_DIRS}
                            ${Eigen_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIR}
                            ${CMAKE_HOME_DIRECTORY}/bivstar
)

add_test(NAME basic-test COMMAND basic-test)

add_executable(ompl-benchmark 
                  ompl-benchmark/ompl-benchmark.cpp
                  bivstar/BIVstar.cpp
                  bivstar/src/ImplicitGraph.cpp
                  bivstar/src/SearchQueue.cpp
                  bivstar/src/Vertex.cpp
)

target_include_directories(ompl-benchmark
                            PRIVATE
                            ${OMPL_INCLUDE_DIRS}
                            ${Eigen_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIR}
                            ${CMAKE_HOME_DIRECTORY}/bivstar
)


target_link_libraries(ompl-benchmark
                        PRIVATE
                        ${OMPL_LIBRARIES}
                        ${Boost_LIBRARIES}
)