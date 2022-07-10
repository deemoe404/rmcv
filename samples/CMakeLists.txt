add_executable(standard standard.cpp)

target_include_directories(standard PRIVATE ../include)
target_link_libraries(standard rmcv)