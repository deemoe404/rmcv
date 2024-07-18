add_executable(standard standard/main.cpp standard/main.h)

target_include_directories(standard PRIVATE ../include)
target_link_libraries(standard rmcv)