cmake_minimum_required(VERSION 3.24)
project(bricks)

include(FetchContent)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O3)

FetchContent_Declare(
    glfw
    URL https://github.com/glfw/glfw/archive/refs/tags/3.4.tar.gz
)
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(imgui_external
    URL https://github.com/ocornut/imgui/archive/refs/tags/v1.92.5.tar.gz
	https://github.com/ocornut/imgui/archive/refs/tags/v1.90.8.tar.gz
	EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(imgui_external)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
FetchContent_MakeAvailable(googletest)

add_library(imgui
	${imgui_external_SOURCE_DIR}/imgui.cpp
	${imgui_external_SOURCE_DIR}/imgui_draw.cpp
	${imgui_external_SOURCE_DIR}/imgui_tables.cpp
	${imgui_external_SOURCE_DIR}/imgui_widgets.cpp
	${imgui_external_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp
    ${imgui_external_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
)
target_include_directories(imgui PUBLIC ${imgui_external_SOURCE_DIR})
target_link_libraries(imgui PUBLIC -lGL glfw)

add_executable(bricks
    src/main.cpp
    src/glad.c
    src/linalg.cpp
    src/physics.cpp
    src/lemke.cpp
    src/render.cpp
)

add_executable(
    bricks_tests 
    src/linalg.cpp
    src/lemke.cpp
    test/test_lcp.cpp
)

add_executable(
    gen_bricks 
    src/gen_bricks.cpp
    src/physics.cpp
    src/linalg.cpp
    src/lemke.cpp
)

target_link_libraries(bricks PRIVATE glfw imgui)
target_include_directories(bricks PRIVATE "./include")
target_include_directories(bricks_tests PRIVATE "./include")
target_include_directories(gen_bricks PRIVATE "./include")
target_link_libraries(bricks_tests PRIVATE gtest gtest_main)
add_test(NAME bricks_tests COMMAND run_tests)
