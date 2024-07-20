cmake_minimum_required(VERSION 3.16.0)
project(rmcv VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
include_directories(
        ./
        ./extra
        ${OpenCV_INCLUDE_DIRS})

aux_source_directory(./src SRC_DIR_CORE)
add_library(rmcv ${SRC_DIR_CORE})
target_link_libraries(rmcv pthread ${OpenCV_LIBS})
set_target_properties(rmcv PROPERTIES LINKER_LANGUAGE CXX)

aux_source_directory(./extra/src SRC_DIR_EXTRA)
add_library(rmcv_extra ${SRC_DIR_EXTRA}
        extra/include/rmcv_extra.h)
target_link_libraries(rmcv_extra gxiapi)
set_target_properties(rmcv_extra PROPERTIES LINKER_LANGUAGE CXX)

add_subdirectory(executable)
