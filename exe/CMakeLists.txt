cmake_minimum_required(VERSION 3.12)
project(runSofaGLFW)

set(SOURCE_FILES
    Main.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} SofaGLFW)
target_link_libraries(${PROJECT_NAME} Sofa.Components.Utils)

target_include_directories(${PROJECT_NAME} PUBLIC extlibs/cxxopts-2.2.1/include)
