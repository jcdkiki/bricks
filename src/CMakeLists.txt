add_subdirectory(editor)
add_subdirectory(linalg)
add_subdirectory(physics)
add_subdirectory(solvers)

add_executable(gen_bricks gen_bricks.cpp)
target_link_libraries(gen_bricks PRIVATE gtest gtest_main physics)
