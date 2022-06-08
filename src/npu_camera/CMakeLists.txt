cmake_minimum_required(VERSION 3.5)
project(npu_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--allow-shlib-undefined")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(RGA_LIBS rga)
set(RKNN_RT_LIB ${CMAKE_SOURCE_DIR}/lib/librknnrt.so)
pkg_check_modules(WIRINGPI REQUIRED libwiringpi2) # /usr/lib/$ARCH/pkg-config/libwiringpi2.pc
pkg_check_modules(GPIOD REQUIRED libgpiod) # /usr/lib/$ARCH/pkg-config/libgpiod.pc

include_directories(${WIRINGPI_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME})

add_executable(camera src/camera.cpp src/postprocess.cc)
ament_target_dependencies(camera geometry_msgs OpenCV PkgConfig rclcpp sensor_msgs std_msgs)

add_executable(operator src/operator.cpp)
ament_target_dependencies(operator geometry_msgs OpenCV PkgConfig rclcpp sensor_msgs std_msgs)

target_link_libraries(camera
    ${OpenCV_LIBS}
    ${RGA_LIBS}
    ${RKNN_RT_LIB}
)

target_link_libraries(operator
    ${GPIOD_LIBRARIES}
    ${OpenCV_LIBS}
    ${WIRINGPI_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install
install(TARGETS
        camera
        operator
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        model
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
