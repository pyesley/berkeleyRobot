cmake_minimum_required(VERSION 3.10)
project(position_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find threads library
find_package(Threads REQUIRED)

# Source files
set(SOURCES
    src/main.cpp
    src/can_interface.cpp
    src/motor_protocol.cpp
    src/tcp_server.cpp
)

# Create executable
add_executable(position_controller ${SOURCES})

# Link libraries
target_link_libraries(position_controller
    Threads::Threads
)

# Include directories
target_include_directories(position_controller PRIVATE src)

# Position test — standalone torque-mode controller with trajectory planner
add_executable(position_test
    src/position_test.cpp
    src/can_interface.cpp
)
target_link_libraries(position_test Threads::Threads)
target_include_directories(position_test PRIVATE src)

# Two-stage position control test (host=position, motor=velocity)
add_executable(position_move_test
    src/position_move_test.cpp
    src/can_interface.cpp
    src/motor_protocol.cpp
)
target_link_libraries(position_move_test Threads::Threads)
target_include_directories(position_move_test PRIVATE src)

# Open-loop motor experiments
add_executable(open_loop_test
    src/open_loop_test.cpp
    src/can_interface.cpp
)
target_link_libraries(open_loop_test Threads::Threads)
target_include_directories(open_loop_test PRIVATE src)
