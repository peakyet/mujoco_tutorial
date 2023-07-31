cmake_minimum_required(VERSION 3.10.0)
project(Tutor)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(mujoco_DIR /home/firefly/mujoco236/lib/cmake/mujoco/)
find_package(mujoco REQUIRED)
if( mujoco_FOUND )
        message("mujoco found!")
        add_executable(main src/main.cpp)
        target_link_libraries(main PRIVATE mujoco::mujoco)
endif(mujoco_FOUND)
