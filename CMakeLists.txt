cmake_minimum_required(VERSION 3.24)
project(bricks)

include(FetchContent)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

FetchContent_Declare(
    glfw
    URL https://github.com/glfw/glfw/archive/refs/tags/3.4.tar.gz
)

FetchContent_MakeAvailable(glfw)

add_executable(bricks
    src/main.c
    src/glad.c
    src/text.c
    src/linalg.c
    src/physics.c
)

set (CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")

target_link_libraries(bricks PRIVATE glfw)
target_include_directories(bricks PRIVATE "./include")
