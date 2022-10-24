# CMakeLists files in this project can
# refer to the root source directory of the project as ${HRPD_SOURCE_DIR} and
# to the root binary directory of the project as ${HRPD_BINARY_DIR}
cmake_minimum_required(VERSION 3.22.1)
project (HRPD)

add_compile_definitions(EIGEN_DONT_PARALLELIZE)

include_directories(include)
find_package(OpenMP)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Libigl
include(libigl)

# Enable the target igl::glfw
igl_include(glfw)

set(OpenMP_CXX_FLAGS "-fopenmp")
set(_CRT_SECURE_NO_WARNINGS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

# Add your project files
file(GLOB SRC_FILES ${HRPD_SOURCE_DIR}/src/*)

add_executable(${PROJECT_NAME} ${SRC_FILES})

#Link igl to project
target_link_libraries(${PROJECT_NAME} PUBLIC igl::glfw)

# Link openMP if found
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()

target_sources(${PROJECT_NAME} PUBLIC ${SRC_FILES})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/armadillo.obj
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
