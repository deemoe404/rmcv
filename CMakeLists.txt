cmake_minimum_required(VERSION 3.16.0)
project(rmcv VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
include_directories(.
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR})
aux_source_directory(src SRC_DIR)
add_library(rmcv ${SRC_DIR})
add_subdirectory(samples)

target_link_libraries(${PROJECT_NAME} gxiapi pthread ${OpenCV_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
