add_executable(
    tests 
    test_lemke.cpp
    test_pg.cpp
    test_linalg.cpp
)
target_link_libraries(tests PRIVATE gtest gtest_main solvers)

add_test(NAME tests COMMAND run_tests)