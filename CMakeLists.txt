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

add_library(
	physics
	src/physics.cpp
	src/linalg.cpp
	src/solvers/lemke.cpp
	src/solvers/pg.cpp
)
target_include_directories(physics PUBLIC ./include)

add_executable(
    tests 
    test/test_lemke.cpp
    test/test_pg.cpp
    test/test_linalg.cpp
)
target_link_libraries(tests PRIVATE gtest gtest_main physics)

add_executable(
	editor
	src/editor/glad.c
	src/editor/main.cpp
	src/editor/render.cpp
)
target_link_libraries(editor PRIVATE imgui glfw physics)

add_executable(
	gen_bricks
	src/gen_bricks.cpp
)
target_link_libraries(gen_bricks PRIVATE physics)

add_test(NAME tests COMMAND run_tests)