cmake_minimum_required(VERSION 3.12)
project(SofaGLFW LANGUAGES CXX VERSION 1.0)

find_package(Sofa.Config REQUIRED)

find_package(SofaSimulation REQUIRED)
find_package(Sofa.GL REQUIRED)
find_package(Sofa.Components.GUI.Core QUIET) # will always be false for now (no package system)

include(FetchContent)

FetchContent_Declare(glfw
        GIT_REPOSITORY https://github.com/glfw/glfw
        GIT_TAG        3.3.4
)

FetchContent_GetProperties(glfw)
if(NOT glfw_POPULATED)
    FetchContent_Populate(glfw)

    set(GLFW_BUILD_EXAMPLES OFF CACHE INTERNAL "Build the GLFW example programs")
    set(GLFW_BUILD_TESTS OFF CACHE INTERNAL "Build the GLFW test programs")
    set(GLFW_BUILD_DOCS OFF CACHE INTERNAL "Build the GLFW documentation")
    set(GLFW_INSTALL OFF CACHE INTERNAL "Generate installation target")

    add_subdirectory(${glfw_SOURCE_DIR} ${glfw_BINARY_DIR})
endif()

set(SOFAGLFW_SOURCE_DIR src/SofaGLFW)

set(HEADER_FILES
	${SOFAGLFW_SOURCE_DIR}/config.h.in
    ${SOFAGLFW_SOURCE_DIR}/SofaGLFWBaseGUI.h
	${SOFAGLFW_SOURCE_DIR}/SofaGLFWWindow.h
)

set(SOURCE_FILES
    ${SOFAGLFW_SOURCE_DIR}/initSofaGLFW.cpp
    ${SOFAGLFW_SOURCE_DIR}/SofaGLFWBaseGUI.cpp
	${SOFAGLFW_SOURCE_DIR}/SofaGLFWWindow.cpp
)

if(Sofa.Components.GUI.Core_FOUND)
    LIST(APPEND HEADER_FILES ${SOFAGLFW_SOURCE_DIR}/SofaGLFWGUI.h)
    LIST(APPEND SOURCE_FILES ${SOFAGLFW_SOURCE_DIR}/SofaGLFWGUI.cpp)
endif()

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

configure_file(${SOFAGLFW_SOURCE_DIR}/config.h.in ${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}/config.h)

target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.GL SofaSimulationGraph)
target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.Components.Visual_minimal)
target_link_libraries(${PROJECT_NAME} PRIVATE glfw)

if(Sofa.Components.GUI.Core_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.Components.GUI.Core)
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE SOFA_BUILD_SOFAGLFW)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/")
