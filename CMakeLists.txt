cmake_minimum_required(VERSION 3.20)
project(BatchInformedVines VERSION 0.1
                           DESCRIPTION "Implement of BITstar algorithm based on OMPL"
                           LANGUAGES CXX)
                
                           
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Boost REQUIRED COMPONENTS program_options)
                           
                           
                           if(${Boost_FOUND})
                               message(STATUS "Found Boost v${Boost_VERSION}")
                           endif()

add_subdirectory(export/ompl export/ompl/build)

# 第一个 target: demo
add_executable(ompl-demo 
                  ompl-demo/ompl-demo.cpp
)

target_include_directories(ompl-demo
                            PUBLIC
                            ompl
)


target_link_libraries(ompl-demo
                        PUBLIC
                        ompl
)

add_executable(basic-test
                bivstar/tests/testPCA.cpp
)

target_include_directories(basic-test
                            PUBLIC
                            ompl
)

add_test(NAME basic-test COMMAND basic-test)

add_executable(ompl-benchmark 
                  ompl-benchmark/ompl-benchmark.cpp
)

target_include_directories(ompl-benchmark
                            PRIVATE
                            ompl
)


target_link_libraries(ompl-benchmark
                        PRIVATE
                        ompl
)

add_executable(circle-grid-benchmark 
                  circle-grid-benchmark/circle_grid.cpp
)

target_include_directories(circle-grid-benchmark 
                            PUBLIC
                            ompl
)


target_link_libraries(circle-grid-benchmark
                        PUBLIC
                        ompl
                        ${Boost_LIBRARIES}
)