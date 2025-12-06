add_library(solvers
    lemke.cpp
    pg.cpp
)
target_include_directories(solvers PUBLIC "${CMAKE_SOURCE_DIR}/include/solvers")
target_link_libraries(solvers PUBLIC linalg)