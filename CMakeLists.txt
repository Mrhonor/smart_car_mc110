cmake_minimum_required(VERSION 3.0.2)
project(smart_car_mc110)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    message_runtime
  DEPENDS
    ${LIBS}
)

include_directories(
  include
  include/main
  include/single_eye
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(smart_car_main
    src/main/crcThings.cpp
    src/main/smart_car_communicator.cpp
    src/main/smart_car_controller.cpp
    src/main/smart_car_loop_queue.cpp
    src/main/smart_car_main.cpp
    src/main/smart_car_protocol.cpp
    src/main/smart_car_uart.cpp
)
add_dependencies(smart_car_main ${${PROJECT_NAME}_EXPORTTED_TARGETS}
                                   ${catkin_EXPORTED_TARGETS}
                                   )

target_link_libraries(smart_car_main 
    ${catkin_LIBRARIES}
)

add_executable(smart_car_single_eye
    src/single_eye/single_eye_body.cpp
    src/single_eye/single_eye_main.cpp
)
add_dependencies(smart_car_single_eye ${${PROJECT_NAME}_EXPORTTED_TARGETS}
                                   ${catkin_EXPORTED_TARGETS}
                                   )

target_link_libraries(smart_car_single_eye 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)


