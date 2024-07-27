add_executable(standard standard/main.cpp standard/main.h)
target_include_directories(standard PRIVATE ../include)
target_link_libraries(standard rmcv)

add_executable(svm_optimizer svm/optimizer.cpp)
target_link_libraries(svm_optimizer rmcv)

add_executable(svm_labeler svm/labeler.cpp)
target_link_libraries(svm_labeler rmcv)

add_executable(kalman_filter_test kalman_filter/test.cpp)
target_link_libraries(kalman_filter_test rmcv)

add_executable(hand_eye_calibration calibration/hand_eye.cpp)
target_link_libraries(hand_eye_calibration rmcv) #rmcv_hardware
