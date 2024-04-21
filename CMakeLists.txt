cmake_minimum_required(VERSION 3.20)
project(BatchInformedVines VERSION 0.1)

# 基础准备
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ompl 1.6 REQUIRED)

if(${OMPL_FOUND})
    message("Found OMPL v${OMPL_VERSION}")
endif()

# 第一个 target: demo
add_executable(ompl-demo 
                  ompl-demo/ompl-demo.cpp
)

target_include_directories(ompl-demo
                            PRIVATE
                            ${OMPL_INCLUDE_DIRS}
)

target_link_libraries(ompl-demo
                        PRIVATE
                        ${OMPL_LIBRARIES}
)
