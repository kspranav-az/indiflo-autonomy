cmake_minimum_required(VERSION 3.1)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

project(stereo_viewer)

# Find OpenCV
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# Executables
add_executable(stereo_viewer stereo_viewer.cpp)
add_executable(stereo_bw_depth stereo_bw_depth.cpp)
add_executable(stereo_metric_depth stereo_metric_depth.cpp)
add_executable(stereo_sgbm_depth stereo_sgbm_depth.cpp)
add_executable(stereo_sgbm_fast stereo_sgbm_fast.cpp)
add_executable(calibrate_stereo calibrate_stereo.cpp)
add_executable(diag_board_size diag_board_size.cpp)
add_executable(calibrate_offline calibrate_offline.cpp)

# Link with OpenCV
target_link_libraries(stereo_viewer PRIVATE ${OpenCV_LIBS})
target_link_libraries(stereo_bw_depth PRIVATE ${OpenCV_LIBS})
target_link_libraries(stereo_metric_depth PRIVATE ${OpenCV_LIBS})
target_link_libraries(stereo_sgbm_depth PRIVATE ${OpenCV_LIBS})
target_link_libraries(stereo_sgbm_fast PRIVATE ${OpenCV_LIBS})
target_link_libraries(calibrate_stereo PRIVATE ${OpenCV_LIBS})
target_link_libraries(diag_board_size PRIVATE ${OpenCV_LIBS})
target_link_libraries(calibrate_offline PRIVATE ${OpenCV_LIBS})
